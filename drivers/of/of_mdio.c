/*
 * OF helpers for the MDIO (Ethernet PHY) API
 *
 * Copyright (c) 2009 Secret Lab Technologies, Ltd.
 *
 * This file is released under the GPLv2
 *
 * This file provides helper functions for extracting PHY device information
 * out of the OpenFirmware device tree and using it to populate an mii_bus.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/err.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/module.h>

MODULE_AUTHOR("Grant Likely <grant.likely@secretlab.ca>");
MODULE_LICENSE("GPL");

static void of_set_phy_supported(struct phy_device *phydev, u32 max_speed)
{
	/* The default values for phydev->supported are provided by the PHY
	 * driver "features" member, we want to reset to sane defaults fist
	 * before supporting higher speeds.
	 */
	phydev->supported &= PHY_DEFAULT_FEATURES;

	switch (max_speed) {
	default:
		return;

	case SPEED_1000:
		phydev->supported |= PHY_1000BT_FEATURES;
	case SPEED_100:
		phydev->supported |= PHY_100BT_FEATURES;
	case SPEED_10:
		phydev->supported |= PHY_10BT_FEATURES;
	}
}

/* Extract the clause 22 phy ID from the compatible string of the form
 * ethernet-phy-idAAAA.BBBB */
static int of_get_phy_id(struct device_node *device, u32 *phy_id)
{
	struct property *prop;
	const char *cp;
	unsigned int upper, lower;

	of_property_for_each_string(device, "compatible", prop, cp) {
		if (sscanf(cp, "ethernet-phy-id%4x.%4x", &upper, &lower) == 2) {
			*phy_id = ((upper & 0xFFFF) << 16) | (lower & 0xFFFF);
			return 0;
		}
	}
	return -EINVAL;
}

static int of_mdiobus_register_phy(struct mii_bus *mdio, struct device_node *child,
				   u32 addr)
{
	struct phy_device *phy;
	bool is_c45;
	int rc;
	u32 max_speed = 0;
	u32 phy_id;

	is_c45 = of_device_is_compatible(child,
					 "ethernet-phy-ieee802.3-c45");

	if (!is_c45 && !of_get_phy_id(child, &phy_id))
		phy = phy_device_create(mdio, addr, phy_id, 0, NULL);
	else
		phy = get_phy_device(mdio, addr, is_c45);
	if (!phy || IS_ERR(phy))
		return 1;

	rc = irq_of_parse_and_map(child, 0);
	if (rc > 0) {
		phy->irq = rc;
		if (mdio->irq)
			mdio->irq[addr] = rc;
	} else {
		if (mdio->irq)
			phy->irq = mdio->irq[addr];
	}

	/* Associate the OF node with the device structure so it
	 * can be looked up later */
	of_node_get(child);
	phy->dev.of_node = child;

	/* All data is now stored in the phy struct;
	 * register it */
	rc = phy_device_register(phy);
	if (rc) {
		phy_device_free(phy);
		of_node_put(child);
		return 1;
	}

	/* Set phydev->supported based on the "max-speed" property
	 * if present */
	if (!of_property_read_u32(child, "max-speed", &max_speed))
		of_set_phy_supported(phy, max_speed);

	dev_dbg(&mdio->dev, "registered phy %s at address %i\n",
		child->name, addr);

	return 0;
}

/**
 * of_mdiobus_register - Register mii_bus and create PHYs from the device tree
 * @mdio: pointer to mii_bus structure
 * @np: pointer to device_node of MDIO bus.
 *
 * This function registers the mii_bus structure and registers a phy_device
 * for each child node of @np.
 */
int of_mdiobus_register(struct mii_bus *mdio, struct device_node *np)
{
	struct device_node *child;
	const __be32 *paddr;
	u32 addr;
	bool scanphys = false;
	int rc, i, len;

	/* Mask out all PHYs from auto probing.  Instead the PHYs listed in
	 * the device tree are populated after the bus has been registered */
	mdio->phy_mask = ~0;

	/* Clear all the IRQ properties */
	if (mdio->irq)
		for (i=0; i<PHY_MAX_ADDR; i++)
			mdio->irq[i] = PHY_POLL;

	mdio->dev.of_node = np;

	/* Register the MDIO bus */
	rc = mdiobus_register(mdio);
	if (rc)
		return rc;

	/* Loop over the child nodes and register a phy_device for each one */
	for_each_available_child_of_node(np, child) {
		/* A PHY must have a reg property in the range [0-31] */
		paddr = of_get_property(child, "reg", &len);
		if (!paddr || len < sizeof(*paddr)) {
			scanphys = true;
			dev_err(&mdio->dev, "%s has invalid PHY address\n",
				child->full_name);
			continue;
		}

		addr = be32_to_cpup(paddr);
		if (addr >= PHY_MAX_ADDR) {
			dev_err(&mdio->dev, "%s PHY address %i is too large\n",
				child->full_name, addr);
			continue;
		}

		rc = of_mdiobus_register_phy(mdio, child, addr);
		if (rc)
			continue;
	}

	if (!scanphys)
		return 0;

	/* auto scan for PHYs with empty reg property */
	for_each_available_child_of_node(np, child) {
		/* Skip PHYs with reg property set */
		paddr = of_get_property(child, "reg", &len);
		if (paddr)
			continue;

		for (addr = 0; addr < PHY_MAX_ADDR; addr++) {
			/* skip already registered PHYs */
			if (mdio->phy_map[addr])
				continue;

			/* be noisy to encourage people to set reg property */
			dev_info(&mdio->dev, "scan phy %s at address %i\n",
				 child->name, addr);

			rc = of_mdiobus_register_phy(mdio, child, addr);
			if (rc)
				continue;
		}
	}

	return 0;
}
EXPORT_SYMBOL(of_mdiobus_register);

/* Helper function for of_phy_find_device */
static int of_phy_match(struct device *dev, void *phy_np)
{
	return dev->of_node == phy_np;
}

/**
 * of_phy_find_device - Give a PHY node, find the phy_device
 * @phy_np: Pointer to the phy's device tree node
 *
 * Returns a pointer to the phy_device.
 */
struct phy_device *of_phy_find_device(struct device_node *phy_np)
{
	struct device *d;
	if (!phy_np)
		return NULL;

	d = bus_find_device(&mdio_bus_type, NULL, phy_np, of_phy_match);
	return d ? to_phy_device(d) : NULL;
}
EXPORT_SYMBOL(of_phy_find_device);

/**
 * of_phy_connect - Connect to the phy described in the device tree
 * @dev: pointer to net_device claiming the phy
 * @phy_np: Pointer to device tree node for the PHY
 * @hndlr: Link state callback for the network device
 * @iface: PHY data interface type
 *
 * Returns a pointer to the phy_device if successful.  NULL otherwise
 */
struct phy_device *of_phy_connect(struct net_device *dev,
				  struct device_node *phy_np,
				  void (*hndlr)(struct net_device *), u32 flags,
				  phy_interface_t iface)
{
	struct phy_device *phy = of_phy_find_device(phy_np);

	if (!phy)
		return NULL;

	return phy_connect_direct(dev, phy, hndlr, iface) ? NULL : phy;
}
EXPORT_SYMBOL(of_phy_connect);

/**
 * of_phy_connect_fixed_link - Parse fixed-link property and return a dummy phy
 * @dev: pointer to net_device claiming the phy
 * @hndlr: Link state callback for the network device
 * @iface: PHY data interface type
 *
 * This function is a temporary stop-gap and will be removed soon.  It is
 * only to support the fs_enet, ucc_geth and gianfar Ethernet drivers.  Do
 * not call this function from new drivers.
 */
struct phy_device *of_phy_connect_fixed_link(struct net_device *dev,
					     void (*hndlr)(struct net_device *),
					     phy_interface_t iface)
{
	struct device_node *net_np;
	char bus_id[MII_BUS_ID_SIZE + 3];
	struct phy_device *phy;
	const __be32 *phy_id;
	int sz;

	if (!dev->dev.parent)
		return NULL;

	net_np = dev->dev.parent->of_node;
	if (!net_np)
		return NULL;

	phy_id = of_get_property(net_np, "fixed-link", &sz);
	if (!phy_id || sz < sizeof(*phy_id))
		return NULL;

	sprintf(bus_id, PHY_ID_FMT, "fixed-0", be32_to_cpu(phy_id[0]));

	phy = phy_connect(dev, bus_id, hndlr, iface);
	return IS_ERR(phy) ? NULL : phy;
}
EXPORT_SYMBOL(of_phy_connect_fixed_link);

/**
 * of_phy_attach - Attach to a PHY without starting the state machine
 * @dev: pointer to net_device claiming the phy
 * @phy_np: Node pointer for the PHY
 * @flags: flags to pass to the PHY
 * @iface: PHY data interface type
 */
struct phy_device *of_phy_attach(struct net_device *dev,
				 struct device_node *phy_np, u32 flags,
				 phy_interface_t iface)
{
	struct phy_device *phy = of_phy_find_device(phy_np);

	if (!phy)
		return NULL;

	return phy_attach_direct(dev, phy, flags, iface) ? NULL : phy;
}
EXPORT_SYMBOL(of_phy_attach);
