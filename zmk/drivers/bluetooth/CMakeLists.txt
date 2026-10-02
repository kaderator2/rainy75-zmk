zephyr_library()
zephyr_library_sources_ifdef(CONFIG_BT_HCI_B91 hci_b91.c b91_mac.c)
zephyr_library_sources_ifdef(CONFIG_BT_HCI_B91_CTLR_BLOB b91_bt.c)

if(CONFIG_BT_HCI_B91_CTLR_OPEN)
  set(B91_DRV ${ZEPHYR_HAL_TELINK_MODULE_DIR}/tlsr9/drivers/B91)
  zephyr_library_sources(
    openll/ll_glue.c
    openll/ll_hci.c
    openll/ll_pdu.c
    openll/ll_adv.c
    openll/ll_radio.c
    openll/ll_sched.c
    ${B91_DRV}/rf.c
  )
  if(NOT CONFIG_ENTROPY_TELINK_B91_TRNG)
    zephyr_library_sources(${B91_DRV}/trng.c)
  endif()
endif()
