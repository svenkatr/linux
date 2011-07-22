#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

static LIST_HEAD(irq_domain_list);
static DEFINE_MUTEX(irq_domain_mutex);

/**
 * irq_domain_add() - Register an irq_domain
 * @domain: ptr to initialized irq_domain structure
 *
 * Registers an irq_domain structure.  The irq_domain must at a minimum be
 * initialized with an ops structure pointer, and either a ->to_irq hook or
 * a valid irq_base value.  Everything else is optional.
 */
void irq_domain_add(struct irq_domain *domain)
{
	mutex_lock(&irq_domain_mutex);
	list_add(&domain->list, &irq_domain_list);
	mutex_unlock(&irq_domain_mutex);
}

/**
 * irq_domain_del() - Unregister an irq_domain
 * @domain: ptr to registered irq_domain.
 */
void irq_domain_del(struct irq_domain *domain)
{
	mutex_lock(&irq_domain_mutex);
	list_del(&domain->list);
	mutex_unlock(&irq_domain_mutex);
}

/**
 * irq_domain_map() - Allocate and/or increment a reference to a hwirq
 *
 * TODO: Establish a linux irq number mapping for a hardware irq.  If the
 * mapping already exists, then increment the reference count and return the
 * linux irq number.
 *
 * At the moment this function is an empty stub since irq_domain initially
 * only supports the common case of mapping hw irq numbers into a contiguous
 * range of pre-allocated linux irq_descs based at irq_domain->irq_base.  When
 * irq_domains are extended either to support non-contiguous mappings (ie. to
 * support MSI interrupts) or to remove preallocation of all irq_descs (as
 * powerpc does so that irq_descs are only allocated for in-use irq inputs),
 * then this function will be extended to implement the irq_desc allocation
 * and reference counting.
 *
 * Any caller to this function must arrange to also call irq_domain_unmap()
 * if the irq ever becomes unused again.
 */
unsigned int irq_domain_map(struct irq_domain *domain, irq_hw_number_t hwirq)
{
	int irq = irq_domain_to_irq(domain, hwirq);
	struct irq_data *d = irq_get_irq_data(irq);

	d->domain = domain;
	d->hwirq = hwirq;

	return irq;
}

/**
 * irq_domain_unmap() - Release a reference to a hwirq
 *
 * TODO: decrement the reference count on a hardware irq number.  If the ref
 * count reaches zero, then the irq_desc can be freed.
 *
 * At the moment this function is an empty stub.  See the comment on
 * irq_domain_map() for details.
 */
void irq_domain_unmap(struct irq_domain *domain, irq_hw_number_t hwirq)
{
	int irq = irq_domain_to_irq(domain, hwirq);
	struct irq_data *d = irq_get_irq_data(irq);

	d->domain = NULL;
}

#if defined(CONFIG_OF_IRQ) && !defined(CONFIG_PPC)

/*
 * A handful of architectures use NO_IRQ, which must be used to avoid breaking
 * things when using the device tree.  This is intended to be temporary until
 * the remaining NO_IRQ users can be removed
 */
#ifndef NO_IRQ
#define NO_IRQ 0
#endif

/**
 * irq_create_of_mapping() - Map a linux irq number from a DT interrupt spec
 *
 * Used by the device tree interrupt mapping code to translate a device tree
 * interrupt specifier to a valid linux irq number.  Returns either a valid
 * linux IRQ number or 0.
 *
 * When the caller no longer need the irq number returned by this function it
 * should arrange to call irq_dispose_mapping().
 */
unsigned int irq_create_of_mapping(struct device_node *controller,
				   const u32 *intspec, unsigned int intsize)
{
	struct irq_domain *domain;
	irq_hw_number_t hwirq;
	unsigned int irq, type;
	int rc = -ENODEV;

	/* Find a domain which can translate the irq spec */
	mutex_lock(&irq_domain_mutex);
	list_for_each_entry(domain, &irq_domain_list, list) {
		if (!domain->ops->dt_translate)
			continue;
		rc = domain->ops->dt_translate(domain, controller,
					intspec, intsize, &hwirq, &type);
		if (rc == 0)
			break;
	}
	mutex_unlock(&irq_domain_mutex);

#if defined(CONFIG_MIPS) || defined(CONFIG_MICROBLAZE)
	/*
	 * Temporary: preserves current behaviour until mips/microblaze
	 * register an irq_domain
	 */
	if (rc != 0) {
		pr_debug("%s: fallback mapping for irq=%i\n",
			 controller->full_name, irq);
		return intspec[0];
	}
#else
	if (rc != 0)
		return NO_IRQ;
#endif

	irq = irq_domain_map(domain, hwirq);
	if (type != IRQ_TYPE_NONE)
		irq_set_irq_type(irq, type);
	pr_debug("%s: mapped hwirq=%i to irq=%i, flags=%x\n",
		 controller->full_name, (int)hwirq, irq, type);
	return irq;
}
EXPORT_SYMBOL_GPL(irq_create_of_mapping);

/**
 * irq_dispose_mapping() - Discard a mapping created by irq_create_of_mapping()
 * @irq: linux irq number to be discarded
 *
 * Calling this function indicates the caller no longer needs a reference to
 * the linux irq number returned by a prior call to irq_create_of_mapping().
 */
void irq_dispose_mapping(unsigned int irq)
{
	struct irq_data *d = irq_get_irq_data(irq);
	irq_domain_unmap(d->domain, d->hwirq);
}
EXPORT_SYMBOL_GPL(irq_dispose_mapping);

int irq_domain_simple_dt_translate(struct irq_domain *d,
			    struct device_node *controller,
			    const u32 *intspec, unsigned int intsize,
			    irq_hw_number_t *out_hwirq, unsigned int *out_type)
{
	if (d->of_node != controller)
		return -EINVAL;
	if (intsize != 1)
		return -EINVAL;

	*out_hwirq = intspec[0];
	*out_type = IRQ_TYPE_NONE;
	return 0;
}

struct irq_domain_ops irq_domain_simple_ops = {
	.dt_translate = irq_domain_simple_dt_translate,
};

/**
 * irq_domain_create_simple() - Set up a 'simple' translation range
 */
void irq_domain_add_simple(struct device_node *controller, int irq_base)
{
	struct irq_domain *domain;

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain) {
		WARN_ON(1);
		return;
	}

	domain->irq_base = irq_base;
	domain->of_node = of_node_get(controller);
	domain->ops = &irq_domain_simple_ops;
	irq_domain_add(domain);
}
EXPORT_SYMBOL_GPL(irq_domain_add_simple);

void irq_domain_generate_simple(const struct of_device_id *match,
				u64 phys_base, unsigned int irq_start)
{
	struct device_node *node;
	pr_info("looking for phys_base=%llx, irq_start=%i\n",
		(unsigned long long) phys_base, (int) irq_start);
	node = of_find_matching_node_by_address(NULL, match, phys_base);
	if (node)
		irq_domain_add_simple(node, irq_start);
	else
		pr_info("no node found\n");
}
EXPORT_SYMBOL_GPL(irq_domain_generate_simple);
#endif
