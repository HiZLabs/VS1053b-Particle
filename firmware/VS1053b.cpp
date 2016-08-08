/*
 * VS1053b.cpp
 *
 *  Created on: Jul 25, 2016
 *      Author: aaron
 */

#include "VS1053b.h"



const unsigned short vs1053b_patches[] = {
#define SKIP_PLUGIN_VARNAME
#include "vs1053b_patches.c.inc"
};

const size_t vs1053b_patch_len = sizeof(vs1053b_patches);

