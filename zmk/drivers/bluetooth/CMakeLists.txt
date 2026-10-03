zephyr_library()
zephyr_library_sources_ifdef(CONFIG_BT_HCI_B91 hci_b91.c b91_mac.c)
zephyr_library_sources_ifdef(CONFIG_BT_HCI_B91_CTLR_BLOB b91_bt.c)

if(CONFIG_BT_HCI_B91_CTLR_OPEN)
  set(B91_DRV ${ZEPHYR_HAL_TELINK_MODULE_DIR}/tlsr9/drivers/B91)
  # openll/*.c include "b91_bt.h" / "b91_mac.h" (one directory up) by quoted
  # path; add this directory so that relative include resolves.
  zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  zephyr_library_sources(
    openll/ll_glue.c
    openll/ll_hci.c
    openll/ll_pdu.c
    openll/ll_adv.c
    openll/ll_radio.c
    openll/ll_radio_mode.c
    openll/ll_sched.c
    openll/ll_arb.c
    openll/ll_csa1.c
    openll/ll_crypt.c
    openll/ll_txq.c
    openll/ll_rxq.c
    openll/ll_conn.c
    openll/ll_credit.c
    openll/ll_llcp.c
    ${B91_DRV}/rf.c
  )
  # hal AES (link encryption, ll_plat_aes_ecb). hal_telink compiles aes.c only
  # with its own CONFIG_BT_B91 BLE path, which neither build uses, so there is
  # no duplicate; the blob build does not need it.
  if(NOT CONFIG_BT_B91)
    zephyr_library_sources(${B91_DRV}/aes.c)
  endif()
  if(NOT CONFIG_ENTROPY_TELINK_B91_TRNG)
    zephyr_library_sources(${B91_DRV}/trng.c)
  endif()
endif()
