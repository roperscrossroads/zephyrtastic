# SPDX-License-Identifier: Apache-2.0

# No board-specific C sources needed. Unlike the Heltec V4 (heltec_wifi_lora32_v4_fem.c,
# runtime FEM detection), this board has no external RF front-end module -- a bare
# SX1262 -- and its GPIO expander is handled entirely by the generic upstream
# nxp,pca9539-compatible driver (CONFIG_GPIO_PCA_SERIES) via devicetree, not board C
# code. See sensecap_indicator_d1l_procpu.dts for both.
