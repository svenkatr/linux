#!/bin/bash
. ./scripts/kbuild-env.sh
kbuild
kbuild_modules_install || true
