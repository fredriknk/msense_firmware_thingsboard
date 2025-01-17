#define CONFIG_APP_VERSION "1.0.0"
#define CONFIG_CLOUD_CONNECTION_RETRY_TIMEOUT_SECONDS 30
#define CONFIG_CLOUD_READY_TIMEOUT_SECONDS 600
#define CONFIG_DATE_TIME_ESTABLISHMENT_TIMEOUT_SECONDS 300
#define CONFIG_APPLICATION_THREAD_STACK_SIZE 2048
#define CONFIG_CONNECTION_THREAD_STACK_SIZE 2048
#define CONFIG_MESSAGE_THREAD_STACK_SIZE 2048
#define CONFIG_LED_THREAD_STACK_SIZE 1024
#define CONFIG_MAX_OUTGOING_MESSAGES 25
#define CONFIG_MAX_CONSECUTIVE_SEND_FAILURES 5
#define CONFIG_CONSECUTIVE_SEND_FAILURE_COOLDOWN_SECONDS 10
#define CONFIG_SENSOR_SAMPLE_INTERVAL_SECONDS 120
#define CONFIG_LOCATION_TRACKING 1
#define CONFIG_LOCATION_TRACKING_SAMPLE_INTERVAL_SECONDS 43200
#define CONFIG_LOCATION_TRACKING_CELLULAR 1
#define CONFIG_TEMP_DATA_USE_SENSOR 1
#define CONFIG_TEMP_TRACKING 1
#define CONFIG_TEMP_ALERT_LIMIT 30
#define CONFIG_LED_INDICATION_DISABLED 1
#define CONFIG_GNSS_FIX_TIMEOUT_SECONDS 40
#define CONFIG_TEST_COUNTER_MULTIPLIER 1
#define CONFIG_AT_CMD_REQUEST_RESPONSE_BUFFER_LENGTH 200
#define CONFIG_POST_PROVISIONING_INTERVAL_M 30
#define CONFIG_I2C 1
#define CONFIG_NET_IPV6 1
#define CONFIG_FLASH_LOAD_SIZE 0x30000
#define CONFIG_FLASH_LOAD_OFFSET 0x50000
#define CONFIG_REGULATOR 1
#define CONFIG_NUM_IRQS 65
#define CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC 32768
#define CONFIG_FLASH_SIZE 1024
#define CONFIG_FLASH_BASE_ADDRESS 0x0
#define CONFIG_MP_MAX_NUM_CPUS 1
#define CONFIG_SOC_RESET_HOOK 1
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 32768
#define CONFIG_ROM_START_OFFSET 0x0
#define CONFIG_PINCTRL 1
#define CONFIG_BUILD_OUTPUT_BIN 1
#define CONFIG_XIP 1
#define CONFIG_MAIN_STACK_SIZE 2048
#define CONFIG_IDLE_STACK_SIZE 320
#define CONFIG_HAS_FLASH_LOAD_OFFSET 1
#define CONFIG_CPU_HAS_ARM_MPU 1
#define CONFIG_PM_DEVICE 1
#define CONFIG_TICKLESS_KERNEL 1
#define CONFIG_FPU 1
#define CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE 4096
#define CONFIG_CLOCK_CONTROL 1
#define CONFIG_CLOCK_CONTROL_INIT_PRIORITY 30
#define CONFIG_USE_DT_CODE_PARTITION 1
#define CONFIG_BUILD_WITH_TFM 1
#define CONFIG_GEN_IRQ_VECTOR_TABLE 1
#define CONFIG_GEN_ISR_TABLES 1
#define CONFIG_TIMESLICE_SIZE 0
#define CONFIG_PM_DEVICE_SYSTEM_MANAGED 1
#define CONFIG_FLASH_FILL_BUFFER_SIZE 32
#define CONFIG_HW_STACK_PROTECTION 1
#define CONFIG_NET_UDP_CHECKSUM 1
#define CONFIG_MFD 1
#define CONFIG_GPIO 1
#define CONFIG_MBEDTLS 1
#define CONFIG_KERNEL_ENTRY "__start"
#define CONFIG_DCACHE_LINE_SIZE 32
#define CONFIG_SOC "nrf9160"
#define CONFIG_ARCH_SW_ISR_TABLE_ALIGN 4
#define CONFIG_NRF_RTC_TIMER 1
#define CONFIG_ASSERT 1
#define CONFIG_BUILD_OUTPUT_HEX 1
#define CONFIG_PM_DEVICE_POWER_DOMAIN 1
#define CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT 1
#define CONFIG_SOC_HAS_TIMING_FUNCTIONS 1
#define CONFIG_ENTROPY_INIT_PRIORITY 50
#define CONFIG_SOC_SERIES "nrf91"
#define CONFIG_SOC_FAMILY "nordic_nrf"
#define CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE -1
#define CONFIG_GEN_SW_ISR_TABLE 1
#define CONFIG_REBOOT 1
#define CONFIG_FLASH_INIT_PRIORITY 50
#define CONFIG_GEN_IRQ_START_VECTOR 0
#define CONFIG_SRAM_OFFSET 0x0
#define CONFIG_CONSOLE 1
#define CONFIG_ARCH_IRQ_VECTOR_TABLE_ALIGN 4
#define CONFIG_ISR_STACK_SIZE 2048
#define CONFIG_ICACHE_LINE_SIZE 32
#define CONFIG_PRIVILEGED_STACK_SIZE 1024
#define CONFIG_DT_HAS_ARM_ARMV8M_MPU_ENABLED 1
#define CONFIG_DT_HAS_ARM_CORTEX_M33F_ENABLED 1
#define CONFIG_DT_HAS_ARM_V8M_NVIC_ENABLED 1
#define CONFIG_DT_HAS_BOSCH_BME680_ENABLED 1
#define CONFIG_DT_HAS_FIXED_PARTITIONS_ENABLED 1
#define CONFIG_DT_HAS_GPIO_KEYS_ENABLED 1
#define CONFIG_DT_HAS_GPIO_LEDS_ENABLED 1
#define CONFIG_DT_HAS_MMIO_SRAM_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NPM1300_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NPM1300_CHARGER_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NPM1300_GPIO_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NPM1300_LED_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NPM1300_REGULATOR_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_CLOCK_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_DPPIC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_EGU_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_GPIO_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_GPIOTE_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_GPREGRET_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_IPC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_KMU_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_PINCTRL_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_POWER_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_PWM_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_SAADC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_TWIM_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_UARTE_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_VMC_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF_WDT_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF91_FLASH_CONTROLLER_ENABLED 1
#define CONFIG_DT_HAS_NORDIC_NRF91X_REGULATORS_ENABLED 1
#define CONFIG_DT_HAS_PWM_LEDS_ENABLED 1
#define CONFIG_DT_HAS_SOC_NV_FLASH_ENABLED 1
#define CONFIG_DT_HAS_ZEPHYR_BT_HCI_ENTROPY_ENABLED 1
#define CONFIG_DT_HAS_ZEPHYR_PSA_CRYPTO_RNG_ENABLED 1
#define CONFIG_NUM_METAIRQ_PRIORITIES 0
#define CONFIG_NET_PKT_RX_COUNT 4
#define CONFIG_NET_PKT_TX_COUNT 4
#define CONFIG_MBEDTLS_PSA_KEY_SLOT_COUNT 32
#define CONFIG_MPSL_WORK_STACK_SIZE 1024
#define CONFIG_NET_RX_STACK_SIZE 1500
#define CONFIG_NET_TX_STACK_SIZE 1200
#define CONFIG_TFM_MCUBOOT_IMAGE_NUMBER 1
#define CONFIG_TFM_CRYPTO_RNG_MODULE_ENABLED 1
#define CONFIG_TFM_PARTITION_CRYPTO 1
#define CONFIG_TFM_PARTITION_PLATFORM 1
#define CONFIG_HIDE_CHILD_PARENT_CONFIG 1
#define CONFIG_WARN_EXPERIMENTAL 1
#define CONFIG_INIT_ARCH_HW_AT_BOOT 1
#define CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE 4096
#define CONFIG_NCS_SAMPLES_DEFAULTS 1
#define CONFIG_UPDATEABLE_IMAGE_NUMBER 1
#define CONFIG_PM_PARTITION_SIZE_PROVISION 0x280
#define CONFIG_PM_PARTITION_SIZE_B0_IMAGE 0x8000
#define CONFIG_SB_VALIDATION_INFO_MAGIC 0x86518483
#define CONFIG_SB_VALIDATION_POINTER_MAGIC 0x6919b47e
#define CONFIG_SB_VALIDATION_INFO_CRYPTO_ID 1
#define CONFIG_SB_VALIDATION_INFO_VERSION 2
#define CONFIG_SB_VALIDATION_METADATA_OFFSET 0
#define CONFIG_SB_VALIDATE_FW_SIGNATURE 1
#define CONFIG_SYSTEM_WORKQUEUE_PRIORITY -1
#define CONFIG_HEAP_MEM_POOL_SIZE 19000
#define CONFIG_DFU_TARGET 1
#define CONFIG_DFU_TARGET_MCUBOOT 1
#define CONFIG_DFU_TARGET_STREAM 1
#define CONFIG_DFU_TARGET_MODEM_DELTA 1
#define CONFIG_DFU_TARGET_MODEM_TIMEOUT 60
#define CONFIG_PCD_VERSION_PAGE_BUF_SIZE 2046
#define CONFIG_NRF_CLOUD_MQTT 1
#define CONFIG_NRF_CLOUD_PORT 8883
#define CONFIG_NRF_CLOUD_SEND_BLOCKING 1
#define CONFIG_NRF_CLOUD_SEND_TIMEOUT_SEC 60
#define CONFIG_NRF_CLOUD_MQTT_MESSAGE_BUFFER_LEN 256
#define CONFIG_NRF_CLOUD_MQTT_PAYLOAD_BUFFER_LEN 2048
#define CONFIG_NRF_CLOUD_CONNECTION_POLL_THREAD 1
#define CONFIG_NRF_CLOUD_CONNECTION_POLL_THREAD_STACK_SIZE 3072
#define CONFIG_NRF_CLOUD_MQTT_KEEPALIVE 1200
#define CONFIG_NRF_CLOUD_MQTT_UA_WAIT_KEEPALIVE 30
#define CONFIG_NRF_CLOUD_HOST_NAME "mqtt.nrfcloud.com"
#define CONFIG_NRF_CLOUD_SEC_TAG 16842753
#define CONFIG_NRF_CLOUD_CHECK_CREDENTIALS 1
#define CONFIG_NRF_CLOUD_CREDENTIALS_MGMT_MODEM 1
#define CONFIG_NRF_CLOUD_AWS_CA_CERT_SIZE_THRESHOLD 1150
#define CONFIG_NRF_CLOUD_COAP_CA_CERT_SIZE_THRESHOLD 550
#define CONFIG_NRF_CLOUD_CLIENT_ID_SRC_IMEI 1
#define CONFIG_NRF_CLOUD_CLIENT_ID_PREFIX "msens-"
#define CONFIG_NRF_CLOUD_FOTA 1
#define CONFIG_NRF_CLOUD_FOTA_AUTO_START_JOB 1
#define CONFIG_NRF_CLOUD_FOTA_PROGRESS_PCT_INCREMENT 10
#define CONFIG_FOTA_USE_NRF_CLOUD_SETTINGS_AREA 1
#define CONFIG_NRF_CLOUD_FOTA_DOWNLOAD_FRAGMENT_SIZE 1700
#define CONFIG_NRF_CLOUD_FOTA_TRANSPORT_ENABLED 1
#define CONFIG_NRF_CLOUD_FOTA_TYPE_APP_SUPPORTED 1
#define CONFIG_NRF_CLOUD_FOTA_TYPE_MODEM_DELTA_SUPPORTED 1
#define CONFIG_NRF_CLOUD_LOCATION 1
#define CONFIG_NRF_CLOUD_WIFI_LOCATION_ENCODE_OPT_MAC_RSSI 1
#define CONFIG_NRF_CLOUD_ALERT 1
#define CONFIG_NRF_CLOUD_ALERT_SEQ_ALWAYS 1
#define CONFIG_NRF_CLOUD_LOG_DIRECT 1
#define CONFIG_NRF_CLOUD_LOG_OUTPUT_LEVEL 3
#define CONFIG_NRF_CLOUD_LOG_BUF_SIZE 256
#define CONFIG_NRF_CLOUD_LOG_TEXT_LOGGING_ENABLED 1
#define CONFIG_NRF_CLOUD_DEVICE_STATUS_ENCODE_VOLTAGE 1
#define CONFIG_NRF_CLOUD_SEND_DEVICE_STATUS 1
#define CONFIG_NRF_CLOUD_SEND_DEVICE_STATUS_NETWORK 1
#define CONFIG_NRF_CLOUD_SEND_DEVICE_STATUS_SIM 1
#define CONFIG_NRF_CLOUD_SEND_DEVICE_STATUS_CONN_INF 1
#define CONFIG_NRF_CLOUD_SEND_SERVICE_INFO_FOTA 1
#define CONFIG_NRF_CLOUD_SEND_SERVICE_INFO_UI 1
#define CONFIG_NRF_CLOUD_ENABLE_SVC_INF_UI_MAP 1
#define CONFIG_NRF_CLOUD_ENABLE_SVC_INF_UI_RSRP 1
#define CONFIG_NRF_CLOUD_ENABLE_SVC_INF_UI_LOGS 1
#define CONFIG_NRF_CLOUD_ENABLE_SVC_INF_UI_TEMP 1
#define CONFIG_NRF_CLOUD_SEND_SHADOW_INFO 1
#define CONFIG_NRF_CLOUD_PRINT_DETAILS 1
#define CONFIG_NRF_CLOUD_VERBOSE_DETAILS 1
#define CONFIG_DOWNLOAD_CLIENT 1
#define CONFIG_DOWNLOAD_CLIENT_BUF_SIZE 2300
#define CONFIG_DOWNLOAD_CLIENT_HTTP_FRAG_SIZE 1024
#define CONFIG_DOWNLOAD_CLIENT_HTTP_FRAG_SIZE_1024 1
#define CONFIG_DOWNLOAD_CLIENT_STACK_SIZE 4096
#define CONFIG_DOWNLOAD_CLIENT_MAX_HOSTNAME_SIZE 128
#define CONFIG_DOWNLOAD_CLIENT_MAX_FILENAME_SIZE 255
#define CONFIG_DOWNLOAD_CLIENT_TCP_SOCK_TIMEO_MS 30000
#define CONFIG_DOWNLOAD_CLIENT_COAP_MAX_RETRANSMIT_REQUEST_COUNT 4
#define CONFIG_DOWNLOAD_CLIENT_RANGE_REQUESTS 1
#define CONFIG_FOTA_DOWNLOAD 1
#define CONFIG_FOTA_SOCKET_RETRIES 2
#define CONFIG_FOTA_DOWNLOAD_PROGRESS_EVT 1
#define CONFIG_FOTA_CLIENT_AUTOSCHEDULE_UPDATE 1
#define CONFIG_FOTA_DOWNLOAD_MCUBOOT_FLASH_BUF_SZ 512
#define CONFIG_FOTA_DOWNLOAD_FILE_NAME_LENGTH 128
#define CONFIG_FOTA_DOWNLOAD_HOST_NAME_LENGTH 128
#define CONFIG_FOTA_DOWNLOAD_RESOURCE_LOCATOR_LENGTH 512
#define CONFIG_FOTA_DOWNLOAD_SEC_TAG_LIST_SIZE_MAX 5
#define CONFIG_FW_INFO 1
#define CONFIG_FW_INFO_OFFSET 0x200
#define CONFIG_FW_INFO_FIRMWARE_VERSION 1
#define CONFIG_FW_INFO_MAGIC_COMMON 0x281ee6de
#define CONFIG_FW_INFO_MAGIC_FIRMWARE_INFO 0x8fcebb4c
#define CONFIG_FW_INFO_MAGIC_EXT_API 0xb845acea
#define CONFIG_FW_INFO_HARDWARE_ID 91
#define CONFIG_FW_INFO_VERSION 2
#define CONFIG_FW_INFO_CRYPTO_ID 0
#define CONFIG_FW_INFO_MAGIC_COMPATIBILITY_ID 0
#define CONFIG_FW_INFO_MAGIC_LEN 12
#define CONFIG_FW_INFO_VALID_VAL 0x9102FFFF
#define CONFIG_EXT_API_PROVIDE_EXT_API_UNUSED 1
#define CONFIG_MPSL_THREAD_COOP_PRIO 8
#define CONFIG_MPSL_TIMESLOT_SESSION_COUNT 0
#define CONFIG_MPSL_HFCLK_LATENCY 1400
#define CONFIG_PARTITION_MANAGER_ENABLED 1
#define CONFIG_FLASH_MAP_CUSTOM 1
#define CONFIG_SRAM_SIZE 128
#define CONFIG_SRAM_BASE_ADDRESS 0x20020000
#define CONFIG_NRF_TRUSTZONE_FLASH_REGION_SIZE 0x8000
#define CONFIG_NRF_TRUSTZONE_RAM_REGION_SIZE 0x2000
#define CONFIG_PM_PARTITION_SIZE_SETTINGS_STORAGE 0x2000
#define CONFIG_PM_PARTITION_ALIGN_SETTINGS_STORAGE 0x8000
#define CONFIG_PM_SINGLE_IMAGE 1
#define CONFIG_PM_EXTERNAL_FLASH_BASE 0x0
#define CONFIG_PM_EXTERNAL_FLASH_PATH ""
#define CONFIG_PM_EXTERNAL_FLASH_SIZE_BITS 0
#define CONFIG_PM_SRAM_BASE 0x20000000
#define CONFIG_PM_SRAM_SIZE 0x40000
#define CONFIG_NRF_SECURITY 1
#define CONFIG_PSA_WANT_GENERATE_RANDOM 1
#define CONFIG_MBEDTLS_CFG_FILE "nrf-config.h"
#define CONFIG_MBEDTLS_PSA_CRYPTO_CONFIG 1
#define CONFIG_MBEDTLS_PSA_CRYPTO_CONFIG_FILE "nrf-psa-crypto-config.h"
#define CONFIG_MBEDTLS_PSA_CRYPTO_USER_CONFIG_FILE "nrf-psa-crypto-user-config.h"
#define CONFIG_MBEDTLS_PSA_CRYPTO_C 1
#define CONFIG_PSA_CRYPTO_CLIENT 1
#define CONFIG_PSA_CORE_OBERON 1
#define CONFIG_MBEDTLS_PSA_CRYPTO_DRIVERS 1
#define CONFIG_PSA_DEFAULT_OFF 1
#define CONFIG_MBEDTLS_PSA_CRYPTO_CLIENT 1
#define CONFIG_PSA_CRYPTO_DRIVER_OBERON 1
#define CONFIG_PSA_WANT_ALG_HMAC_DRBG 1
#define CONFIG_PSA_USE_HMAC_DRBG_DRIVER 1
#define CONFIG_PSA_USE_CC3XX_HMAC_DRBG_DRIVER 1
#define CONFIG_PSA_WANT_AES_KEY_SIZE_128 1
#define CONFIG_PSA_WANT_AES_KEY_SIZE_192 1
#define CONFIG_PSA_WANT_AES_KEY_SIZE_256 1
#define CONFIG_PSA_MAX_RSA_KEY_BITS 0
#define CONFIG_PSA_ACCEL_GENERATE_RANDOM 1
#define CONFIG_PSA_NEED_CC3XX_HMAC_DRBG_DRIVER 1
#define CONFIG_MBEDTLS_USE_PSA_CRYPTO 1
#define CONFIG_MBEDTLS_PLATFORM_MEMORY 1
#define CONFIG_MBEDTLS_PLATFORM_C 1
#define CONFIG_MBEDTLS_MEMORY_C 1
#define CONFIG_MBEDTLS_MEMORY_BUFFER_ALLOC_C 1
#define CONFIG_MBEDTLS_THREADING_C 1
#define CONFIG_MBEDTLS_BASE64_C 1
#define CONFIG_MBEDTLS_OID_C 1
#define CONFIG_MBEDTLS_ASN1_WRITE_C 1
#define CONFIG_MBEDTLS_ASN1_PARSE_C 1
#define CONFIG_MBEDTLS_ENTROPY_HARDWARE_ALT 1
#define CONFIG_MBEDTLS_AES_SETKEY_ENC_ALT 1
#define CONFIG_MBEDTLS_AES_SETKEY_DEC_ALT 1
#define CONFIG_MBEDTLS_AES_ENCRYPT_ALT 1
#define CONFIG_MBEDTLS_AES_DECRYPT_ALT 1
#define CONFIG_MBEDTLS_CHACHA20_ALT 1
#define CONFIG_MBEDTLS_POLY1305_ALT 1
#define CONFIG_MBEDTLS_ECDH_GEN_PUBLIC_ALT 1
#define CONFIG_MBEDTLS_ECDH_COMPUTE_SHARED_ALT 1
#define CONFIG_MBEDTLS_ECDSA_GENKEY_ALT 1
#define CONFIG_MBEDTLS_ECDSA_SIGN_ALT 1
#define CONFIG_MBEDTLS_ECDSA_VERIFY_ALT 1
#define CONFIG_MBEDTLS_ECJPAKE_ALT 1
#define CONFIG_MBEDTLS_SHA1_ALT 1
#define CONFIG_MBEDTLS_SHA224_ALT 1
#define CONFIG_MBEDTLS_SHA256_ALT 1
#define CONFIG_MBEDTLS_ENTROPY_FORCE_SHA256 1
#define CONFIG_MBEDTLS_ENTROPY_MAX_SOURCES 1
#define CONFIG_MBEDTLS_NO_PLATFORM_ENTROPY 1
#define CONFIG_OBERON_ONLY_PSA_ENABLED 1
#define CONFIG_OBERON_ONLY_ENABLED 1
#define CONFIG_MBEDTLS_MPI_WINDOW_SIZE 6
#define CONFIG_MBEDTLS_MPI_MAX_SIZE 256
#define CONFIG_MBEDTLS_LIBRARY_NRF_SECURITY 1
#define CONFIG_AUDIO_MODULE_NAME_SIZE 20
#define CONFIG_MCUBOOT_APPLICATION_IMAGE_NUMBER 0
#define CONFIG_MCUBOOT_NETWORK_CORE_IMAGE_NUMBER -1
#define CONFIG_MCUBOOT_WIFI_PATCHES_IMAGE_NUMBER -1
#define CONFIG_MCUBOOT_QSPI_XIP_IMAGE_NUMBER -1
#define CONFIG_MCUBOOT_MCUBOOT_IMAGE_NUMBER -1
#define CONFIG_WFA_QT_THREAD_STACK_SIZE 5200
#define CONFIG_WFA_QT_REBOOT_TIMEOUT_MS 1000
#define CONFIG_WFA_QT_DEFAULT_INTERFACE "wlan0"
#define CONFIG_WPAS_READY_TIMEOUT_MS 10000
#define CONFIG_NRF_MODEM_LIB 1
#define CONFIG_NRF_MODEM_LIB_HEAP_SIZE 1024
#define CONFIG_NRF_MODEM_LIB_SHMEM_CTRL_SIZE 0x4e8
#define CONFIG_NRF_MODEM_LIB_SHMEM_TX_SIZE 4096
#define CONFIG_NRF_MODEM_LIB_SHMEM_RX_SIZE 8192
#define CONFIG_NRF_MODEM_LIB_SHMEM_TRACE_SIZE 0
#define CONFIG_NRF_MODEM_LIB_SENDMSG_BUF_SIZE 128
#define CONFIG_NRF_MODEM_LIB_ON_FAULT_LTE_NET_IF 1
#define CONFIG_NRF_MODEM_LIB_CFUN_HOOKS 1
#define CONFIG_NRF_MODEM_LIB_NET_IF 1
#define CONFIG_NRF_MODEM_LIB_NET_IF_WORKQUEUE_STACK_SIZE 1024
#define CONFIG_NRF_MODEM_LIB_NET_IF_CONNECT_TIMEOUT_SECONDS 0
#define CONFIG_NRF_MODEM_LIB_NET_IF_CONNECTION_PERSISTENCE 1
#define CONFIG_NRF_MODEM_LIB_NET_IF_AUTO_DOWN 1
#define CONFIG_NRF_MODEM_LIB_NET_IF_DOWN_DEFAULT_LTE_DISCONNECT 1
#define CONFIG_NET_CONNECTION_MANAGER_MONITOR_STACK_SIZE 1024
#define CONFIG_NRF_MODEM_LIB_IPC_IRQ_PRIO 1
#define CONFIG_AT_MONITOR 1
#define CONFIG_AT_MONITOR_HEAP_SIZE 2048
#define CONFIG_LTE_LINK_CONTROL 1
#define CONFIG_LTE_LC_EDRX_MODULE 1
#define CONFIG_LTE_LC_NEIGHBOR_CELL_MEAS_MODULE 1
#define CONFIG_LTE_LC_PSM_MODULE 1
#define CONFIG_LTE_PLMN_SELECTION_OPTIMIZATION 1
#define CONFIG_LTE_PSM_REQ 1
#define CONFIG_LTE_PSM_REQ_FORMAT_STRING 1
#define CONFIG_LTE_PSM_REQ_RPTAU "00100001"
#define CONFIG_LTE_PSM_REQ_RAT "00000000"
#define CONFIG_LTE_PSM_REQ_RPTAU_SECONDS 1800
#define CONFIG_LTE_PSM_REQ_RAT_SECONDS 60
#define CONFIG_LTE_EDRX_REQ_VALUE_LTE_M "1001"
#define CONFIG_LTE_EDRX_REQ_VALUE_NBIOT "1001"
#define CONFIG_LTE_PTW_VALUE_LTE_M ""
#define CONFIG_LTE_PTW_VALUE_NBIOT ""
#define CONFIG_LTE_NETWORK_MODE_LTE_M_NBIOT_GPS 1
#define CONFIG_LTE_MODE_PREFERENCE_LTE_M_PLMN_PRIO 1
#define CONFIG_LTE_MODE_PREFERENCE_VALUE 3
#define CONFIG_LTE_NETWORK_TIMEOUT 600
#define CONFIG_LTE_NEIGHBOR_CELLS_MAX 10
#define CONFIG_LTE_LC_WORKQUEUE_STACK_SIZE 1024
#define CONFIG_NRF_SPU_FLASH_REGION_SIZE 0x8000
#define CONFIG_FPROTECT_BLOCK_SIZE 0x8000
#define CONFIG_DK_LIBRARY 1
#define CONFIG_DK_LIBRARY_BUTTON_SCAN_INTERVAL 10
#define CONFIG_DK_LIBRARY_DYNAMIC_BUTTON_HANDLERS 1
#define CONFIG_AT_PARSER 1
#define CONFIG_MODEM_INFO 1
#define CONFIG_MODEM_INFO_MAX_AT_PARAMS_RSP 10
#define CONFIG_MODEM_INFO_BUFFER_SIZE 128
#define CONFIG_MODEM_INFO_ADD_NETWORK 1
#define CONFIG_MODEM_INFO_ADD_DATE_TIME 1
#define CONFIG_MODEM_INFO_ADD_SIM 1
#define CONFIG_MODEM_INFO_ADD_SIM_ICCID 1
#define CONFIG_MODEM_INFO_ADD_SIM_IMSI 1
#define CONFIG_MODEM_INFO_ADD_DEVICE 1
#define CONFIG_PDN 1
#define CONFIG_PDN_ESM_TIMEOUT 1000
#define CONFIG_PDN_INIT_PRIORITY 90
#define CONFIG_PDN_ESM_STRERROR 1
#define CONFIG_RESET_ON_FATAL_ERROR 1
#define CONFIG_MODEM_KEY_MGMT 1
#define CONFIG_DATE_TIME 1
#define CONFIG_DATE_TIME_AUTO_UPDATE 1
#define CONFIG_DATE_TIME_UPDATE_INTERVAL_SECONDS 14400
#define CONFIG_DATE_TIME_TOO_OLD_SECONDS 3600
#define CONFIG_DATE_TIME_RETRY_COUNT 0
#define CONFIG_DATE_TIME_RETRY_INTERVAL_SECONDS 60
#define CONFIG_DATE_TIME_MODEM 1
#define CONFIG_DATE_TIME_NTP 1
#define CONFIG_DATE_TIME_THREAD_STACK_SIZE 1280
#define CONFIG_DATE_TIME_NTP_QUERY_TIME_SECONDS 5
#define CONFIG_HW_UNIQUE_KEY_SUPPORTED 1
#define CONFIG_HW_UNIQUE_KEY_PARTITION_SIZE 0x0
#define CONFIG_LOCATION 1
#define CONFIG_LOCATION_METHOD_CELLULAR 1
#define CONFIG_LOCATION_METHODS_LIST_SIZE 3
#define CONFIG_LOCATION_WORKQUEUE_STACK_SIZE 4096
#define CONFIG_LOCATION_SERVICE_CLOUD_RECV_BUF_SIZE 512
#define CONFIG_LOCATION_SERVICE_NRF_CLOUD 1
#define CONFIG_LOCATION_REQUEST_DEFAULT_METHOD_FIRST_CELLULAR 1
#define CONFIG_LOCATION_REQUEST_DEFAULT_METHOD_SECOND_NONE 1
#define CONFIG_LOCATION_REQUEST_DEFAULT_METHOD_THIRD_NONE 1
#define CONFIG_LOCATION_REQUEST_DEFAULT_INTERVAL 0
#define CONFIG_LOCATION_REQUEST_DEFAULT_TIMEOUT 300000
#define CONFIG_LOCATION_REQUEST_DEFAULT_CELLULAR_TIMEOUT 30000
#define CONFIG_LOCATION_REQUEST_DEFAULT_CELLULAR_CELL_COUNT 4
#define CONFIG_NCS_BOOT_BANNER 1
#define CONFIG_NCS_NCS_BOOT_BANNER_STRING "nRF Connect SDK"
#define CONFIG_NCS_ZEPHYR_BOOT_BANNER_STRING "Zephyr OS"
#define CONFIG_SENSOR 1
#define CONFIG_NRFX_GPIOTE_NUM_OF_EVT_HANDLERS 1
#define CONFIG_ZTEST_MULTICORE_DEFAULT_SETTINGS 1
#define CONFIG_ZEPHYR_NRF_MODULE 1
#define CONFIG_BOOT_SIGNATURE_KEY_FILE ""
#define CONFIG_MCUBOOT_FLASH_WRITE_BLOCK_SIZE 4
#define CONFIG_DT_FLASH_WRITE_BLOCK_SIZE 4
#define CONFIG_ZEPHYR_MCUBOOT_MODULE 1
#define CONFIG_ZEPHYR_MBEDTLS_MODULE 1
#define CONFIG_DISABLE_MBEDTLS_BUILTIN 1
#define CONFIG_MBEDTLS_INIT 1
#define CONFIG_APP_LINK_WITH_MBEDTLS 1
#define CONFIG_ZEPHYR_OBERON_PSA_CRYPTO_MODULE 1
#define CONFIG_TFM_BOARD "C:/ncs/v2.9.0/nrf/modules/trusted-firmware-m/tfm_boards/nrf9160"
#define CONFIG_TFM_LOG_LEVEL_SILENCE 1
#define CONFIG_TFM_QCBOR_PATH "C:/ncs/v2.9.0/modules/tee/tf-m/trusted-firmware-m/../qcbor"
#define CONFIG_BOOTLOADER_MCUBOOT 1
#define CONFIG_ZEPHYR_TRUSTED_FIRMWARE_M_MODULE 1
#define CONFIG_TFM_CMAKE_BUILD_TYPE_MINSIZEREL 1
#define CONFIG_TFM_ISOLATION_LEVEL 1
#define CONFIG_TFM_PARTITION_PLATFORM_CUSTOM_REBOOT 1
#define CONFIG_TFM_BL2_NOT_SUPPORTED 1
#define CONFIG_TFM_IMAGE_VERSION_S "0.0.0+0"
#define CONFIG_TFM_IMAGE_VERSION_NS "0.0.0+0"
#define CONFIG_TFM_SFN 1
#define CONFIG_TFM_PSA_TEST_NONE 1
#define CONFIG_TFM_FLASH_MERGED_BINARY 1
#define CONFIG_TFM_EXCEPTION_INFO_DUMP 1
#define CONFIG_TFM_PLATFORM_SERVICE_INPUT_BUFFER_SIZE 64
#define CONFIG_TFM_PLATFORM_SERVICE_OUTPUT_BUFFER_SIZE 64
#define CONFIG_TFM_PLATFORM_SP_STACK_SIZE 0x500
#define CONFIG_TFM_PLATFORM_NV_COUNTER_MODULE_DISABLED 1
#define CONFIG_TFM_CRYPTO_ENGINE_BUF_SIZE 1
#define CONFIG_TFM_CRYPTO_CONC_OPER_NUM 1
#define CONFIG_TFM_CRYPTO_IOVEC_BUFFER_SIZE 1024
#define CONFIG_TFM_CRYPTO_PARTITION_STACK_SIZE 0x800
#define CONFIG_PM_PARTITION_SIZE_TFM_SRAM 0x8000
#define CONFIG_PM_PARTITION_SIZE_TFM 0x7E00
#define CONFIG_PM_PARTITION_SIZE_TFM_PROTECTED_STORAGE 0x0
#define CONFIG_PM_PARTITION_SIZE_TFM_INTERNAL_TRUSTED_STORAGE 0x0
#define CONFIG_PM_PARTITION_SIZE_TFM_OTP_NV_COUNTERS 0x0
#define CONFIG_NRF_GPIO0_PIN_MASK_SECURE 0x00000000
#define CONFIG_NRF_DPPI_CHANNEL_MASK_SECURE 0x00000000
#define CONFIG_NRF_VMC_SECURE 1
#define CONFIG_TFM_PROFILE_TYPE_MINIMAL 1
#define CONFIG_TFM_HW_INIT_RESET_ON_BOOT 1
#define CONFIG_TFM_HW_INIT_NRF_PERIPHERALS 1
#define CONFIG_TFM_ALLOW_NON_SECURE_RESET 1
#define CONFIG_TFM_ALLOW_NON_SECURE_FAULT_HANDLING 1
#define CONFIG_TFM_S_CODE_VECTOR_TABLE_SIZE 0x144
#define CONFIG_ZEPHYR_PSA_ARCH_TESTS_MODULE 1
#define CONFIG_ZEPHYR_SOC_HWMV1_MODULE 1
#define CONFIG_CJSON_LIB 1
#define CONFIG_ZEPHYR_CJSON_MODULE 1
#define CONFIG_ZEPHYR_AZURE_SDK_FOR_C_MODULE 1
#define CONFIG_ZEPHYR_CIRRUS_LOGIC_MODULE 1
#define CONFIG_ZEPHYR_OPENTHREAD_MODULE 1
#define CONFIG_SUIT_ENVELOPE_TEMPLATE_FILENAME ""
#define CONFIG_SUIT_ENVELOPE_TARGET ""
#define CONFIG_SUIT_ENVELOPE_OUTPUT_ARTIFACT "merged.hex"
#define CONFIG_SUIT_ENVELOPE_OUTPUT_MPI_MERGE 1
#define CONFIG_ZEPHYR_SUIT_GENERATOR_MODULE 1
#define CONFIG_SUIT_PLATFORM_DRY_RUN_SUPPORT 1
#define CONFIG_ZEPHYR_SUIT_PROCESSOR_MODULE 1
#define CONFIG_ZEPHYR_MEMFAULT_FIRMWARE_SDK_MODULE 1
#define CONFIG_ZEPHYR_COREMARK_MODULE 1
#define CONFIG_ZEPHYR_CANOPENNODE_MODULE 1
#define CONFIG_ZEPHYR_CHRE_MODULE 1
#define CONFIG_ZEPHYR_LZ4_MODULE 1
#define CONFIG_ZEPHYR_NANOPB_MODULE 1
#define CONFIG_ZEPHYR_TF_M_TESTS_MODULE 1
#define CONFIG_ZEPHYR_ZSCILIB_MODULE 1
#define CONFIG_ZEPHYR_CMSIS_MODULE 1
#define CONFIG_HAS_CMSIS_CORE 1
#define CONFIG_HAS_CMSIS_CORE_M 1
#define CONFIG_ZEPHYR_CMSIS_DSP_MODULE 1
#define CONFIG_ZEPHYR_CMSIS_NN_MODULE 1
#define CONFIG_ZEPHYR_FATFS_MODULE 1
#define CONFIG_ZEPHYR_HAL_NORDIC_MODULE 1
#define CONFIG_HAS_NRFX 1
#define CONFIG_NRFX_CLOCK 1
#define CONFIG_NRFX_CLOCK_LFXO_TWO_STAGE_ENABLED 1
#define CONFIG_NRFX_GPIOTE 1
#define CONFIG_NRFX_GPIOTE1 1
#define CONFIG_NRFX_IPC 1
#define CONFIG_NRFX_NVMC 1
#define CONFIG_NRFX_TWIM 1
#define CONFIG_NRFX_TWIM2 1
#define CONFIG_NRFX_RESERVED_RESOURCES_HEADER "nrfx_config_reserved_resources_ncs.h"
#define CONFIG_ZEPHYR_HAL_ST_MODULE 1
#define CONFIG_ZEPHYR_HAL_WURTHELEKTRONIK_MODULE 1
#define CONFIG_ZEPHYR_HOSTAP_MODULE 1
#define CONFIG_ZEPHYR_LIBMETAL_MODULE 1
#define CONFIG_ZEPHYR_LIBLC3_MODULE 1
#define CONFIG_ZEPHYR_LITTLEFS_MODULE 1
#define CONFIG_ZEPHYR_LORAMAC_NODE_MODULE 1
#define CONFIG_ZEPHYR_LVGL_MODULE 1
#define CONFIG_ZEPHYR_MIPI_SYS_T_MODULE 1
#define CONFIG_ZEPHYR_NRF_WIFI_MODULE 1
#define CONFIG_ZEPHYR_OPEN_AMP_MODULE 1
#define CONFIG_ZEPHYR_PICOLIBC_MODULE 1
#define CONFIG_ZEPHYR_SEGGER_MODULE 1
#define CONFIG_HAS_SEGGER_RTT 1
#define CONFIG_ZEPHYR_TINYCRYPT_MODULE 1
#define CONFIG_ZEPHYR_UOSCORE_UEDHOC_MODULE 1
#define CONFIG_ZEPHYR_ZCBOR_MODULE 1
#define CONFIG_NRF_MODEM 1
#define CONFIG_NRF_MODEM_LINK_BINARY 1
#define CONFIG_NRF_MODEM_LINK_BINARY_CELLULAR 1
#define CONFIG_NRF_MODEM_SHMEM_CTRL_SIZE 0x4e8
#define CONFIG_HAS_HW_NRF_CC3XX 1
#define CONFIG_NRF_802154_SOURCE_NRFXLIB 1
#define CONFIG_ZEPHYR_NRFXLIB_MODULE 1
#define CONFIG_ZEPHYR_NRF_HW_MODELS_MODULE 1
#define CONFIG_ZEPHYR_CONNECTEDHOMEIP_MODULE 1
#define CONFIG_MCUBOOT_CMAKE_WEST_SIGN_PARAMS "--quiet"
#define CONFIG_MCUBOOT_SIGNATURE_KEY_FILE "C:/ncs/v2.9.0/bootloader/mcuboot/root-ec-p256.pem"
#define CONFIG_MCUBOOT_ENCRYPTION_KEY_FILE ""
#define CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION "0.0.0+0"
#define CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS ""
#define CONFIG_MCUBOOT_BOOTLOADER_MODE_SWAP_WITHOUT_SCRATCH 1
#define CONFIG_MCUBOOT_BOOTLOADER_MODE_HAS_NO_DOWNGRADE 1
#define CONFIG_MCUBOOT_BOOTUTIL_LIB 1
#define CONFIG_MCUBOOT_BOOTUTIL_LIB_OWN_LOG 1
#define CONFIG_BOARD "msensedk"
#define CONFIG_BOARD_REVISION "0.14.0"
#define CONFIG_BOARD_TARGET "msensedk@0.14.0/nrf9160/ns"
#define CONFIG_BOARD_MSENSEDK 1
#define CONFIG_BOARD_MSENSEDK_NRF9160_NS 1
#define CONFIG_BOARD_QUALIFIERS "nrf9160/ns"
#define CONFIG_SOC_FAMILY_NORDIC_NRF 1
#define CONFIG_SOC_SERIES_NRF91X 1
#define CONFIG_SOC_NRF9160 1
#define CONFIG_SOC_NRF9160_SICA 1
#define CONFIG_HAS_HW_NRF_CC310 1
#define CONFIG_HAS_HW_NRF_CLOCK 1
#define CONFIG_HAS_HW_NRF_DPPIC 1
#define CONFIG_HAS_HW_NRF_EGU0 1
#define CONFIG_HAS_HW_NRF_EGU1 1
#define CONFIG_HAS_HW_NRF_EGU2 1
#define CONFIG_HAS_HW_NRF_EGU3 1
#define CONFIG_HAS_HW_NRF_EGU4 1
#define CONFIG_HAS_HW_NRF_EGU5 1
#define CONFIG_HAS_HW_NRF_GPIO0 1
#define CONFIG_HAS_HW_NRF_GPIOTE1 1
#define CONFIG_HAS_HW_NRF_KMU 1
#define CONFIG_HAS_HW_NRF_NVMC_PE 1
#define CONFIG_HAS_HW_NRF_POWER 1
#define CONFIG_HAS_HW_NRF_PWM0 1
#define CONFIG_HAS_HW_NRF_SAADC 1
#define CONFIG_HAS_HW_NRF_TWIM2 1
#define CONFIG_HAS_HW_NRF_UARTE0 1
#define CONFIG_HAS_HW_NRF_VMC 1
#define CONFIG_HAS_HW_NRF_WDT0 1
#define CONFIG_NRF_ENABLE_ICACHE 1
#define CONFIG_NRF_SPU_RAM_REGION_SIZE 0x2000
#define CONFIG_NRF_RTC_TIMER_USER_CHAN_COUNT 0
#define CONFIG_NRF_SOC_SECURE_SUPPORTED 1
#define CONFIG_TFM_NRF_NS_STORAGE 1
#define CONFIG_NRF_APPROTECT_USE_UICR 1
#define CONFIG_NRF_SECURE_APPROTECT_USE_UICR 1
#define CONFIG_GPIO_INIT_PRIORITY 40
#define CONFIG_SOC_COMPATIBLE_NRF 1
#define CONFIG_ARCH "arm"
#define CONFIG_ARCH_HAS_SINGLE_THREAD_SUPPORT 1
#define CONFIG_CPU_CORTEX 1
#define CONFIG_KOBJECT_TEXT_AREA 256
#define CONFIG_ARM_MPU 1
#define CONFIG_ARM_MPU_REGION_MIN_ALIGN_AND_SIZE 32
#define CONFIG_MPU_ALLOW_FLASH_WRITE 1
#define CONFIG_CPU_CORTEX_M 1
#define CONFIG_ISA_THUMB2 1
#define CONFIG_ASSEMBLER_ISA_THUMB2 1
#define CONFIG_COMPILER_ISA_THUMB2 1
#define CONFIG_STACK_ALIGN_DOUBLE_WORD 1
#define CONFIG_FAULT_DUMP 2
#define CONFIG_BUILTIN_STACK_GUARD 1
#define CONFIG_ARM_STACK_PROTECTION 1
#define CONFIG_ARM_NONSECURE_FIRMWARE 1
#define CONFIG_ARM_NONSECURE_PREEMPTIBLE_SECURE_CALLS 1
#define CONFIG_ARM_STORE_EXC_RETURN 1
#define CONFIG_FP_HARDABI 1
#define CONFIG_FP16 1
#define CONFIG_FP16_IEEE 1
#define CONFIG_CPU_CORTEX_M33 1
#define CONFIG_CPU_CORTEX_M_HAS_SYSTICK 1
#define CONFIG_CPU_CORTEX_M_HAS_DWT 1
#define CONFIG_CPU_CORTEX_M_HAS_BASEPRI 1
#define CONFIG_CPU_CORTEX_M_HAS_VTOR 1
#define CONFIG_CPU_CORTEX_M_HAS_SPLIM 1
#define CONFIG_CPU_CORTEX_M_HAS_PROGRAMMABLE_FAULT_PRIOS 1
#define CONFIG_CPU_CORTEX_M_HAS_CMSE 1
#define CONFIG_ARMV7_M_ARMV8_M_MAINLINE 1
#define CONFIG_ARMV8_M_MAINLINE 1
#define CONFIG_ARMV8_M_SE 1
#define CONFIG_ARMV7_M_ARMV8_M_FP 1
#define CONFIG_ARMV8_M_DSP 1
#define CONFIG_NULL_POINTER_EXCEPTION_DETECTION_NONE 1
#define CONFIG_ARM_TRUSTZONE_M 1
#define CONFIG_CUSTOM_SECTION_MIN_ALIGN_SIZE 32
#define CONFIG_CPU_HAS_NRF_IDAU 1
#define CONFIG_ARM 1
#define CONFIG_ARCH_IS_SET 1
#define CONFIG_LITTLE_ENDIAN 1
#define CONFIG_TRUSTED_EXECUTION_NONSECURE 1
#define CONFIG_KOBJECT_DATA_AREA_RESERVE_EXTRA_PERCENT 100
#define CONFIG_KOBJECT_RODATA_AREA_EXTRA_BYTES 16
#define CONFIG_GEN_PRIV_STACKS 1
#define CONFIG_ISR_TABLES_LOCAL_DECLARATION_SUPPORTED 1
#define CONFIG_IRQ_VECTOR_TABLE_JUMP_BY_ADDRESS 1
#define CONFIG_EXCEPTION_DEBUG 1
#define CONFIG_ARCH_HAS_TIMING_FUNCTIONS 1
#define CONFIG_ARCH_HAS_TRUSTED_EXECUTION 1
#define CONFIG_ARCH_HAS_STACK_PROTECTION 1
#define CONFIG_ARCH_HAS_USERSPACE 1
#define CONFIG_ARCH_HAS_EXECUTABLE_PAGE_BIT 1
#define CONFIG_ARCH_HAS_RAMFUNC_SUPPORT 1
#define CONFIG_ARCH_HAS_NESTED_EXCEPTION_DETECTION 1
#define CONFIG_ARCH_SUPPORTS_COREDUMP 1
#define CONFIG_ARCH_SUPPORTS_COREDUMP_THREADS 1
#define CONFIG_ARCH_SUPPORTS_ARCH_HW_INIT 1
#define CONFIG_ARCH_SUPPORTS_ROM_START 1
#define CONFIG_ARCH_HAS_EXTRA_EXCEPTION_INFO 1
#define CONFIG_ARCH_HAS_THREAD_LOCAL_STORAGE 1
#define CONFIG_ARCH_HAS_SUSPEND_TO_RAM 1
#define CONFIG_ARCH_HAS_THREAD_ABORT 1
#define CONFIG_ARCH_HAS_CODE_DATA_RELOCATION 1
#define CONFIG_CPU_HAS_TEE 1
#define CONFIG_CPU_HAS_FPU 1
#define CONFIG_CPU_HAS_MPU 1
#define CONFIG_MPU 1
#define CONFIG_MPU_REQUIRES_NON_OVERLAPPING_REGIONS 1
#define CONFIG_MPU_GAP_FILLING 1
#define CONFIG_SRAM_REGION_PERMISSIONS 1
#define CONFIG_FPU_SHARING 1
#define CONFIG_TOOLCHAIN_HAS_BUILTIN_FFS 1
#define CONFIG_ARCH_HAS_CUSTOM_SWAP_TO_MAIN 1
#define CONFIG_MULTITHREADING 1
#define CONFIG_NUM_COOP_PRIORITIES 16
#define CONFIG_NUM_PREEMPT_PRIORITIES 15
#define CONFIG_MAIN_THREAD_PRIORITY 0
#define CONFIG_COOP_ENABLED 1
#define CONFIG_PREEMPT_ENABLED 1
#define CONFIG_PRIORITY_CEILING -127
#define CONFIG_THREAD_STACK_INFO 1
#define CONFIG_SCHED_DUMB 1
#define CONFIG_WAITQ_DUMB 1
#define CONFIG_LIBC_ERRNO 1
#define CONFIG_ERRNO 1
#define CONFIG_CURRENT_THREAD_USE_TLS 1
#define CONFIG_BOOT_DELAY 0
#define CONFIG_BARRIER_OPERATIONS_ARCH 1
#define CONFIG_ATOMIC_OPERATIONS_BUILTIN 1
#define CONFIG_TIMESLICING 1
#define CONFIG_TIMESLICE_PRIORITY 0
#define CONFIG_POLL 1
#define CONFIG_NUM_MBOX_ASYNC_MSGS 10
#define CONFIG_EVENTS 1
#define CONFIG_KERNEL_MEM_POOL 1
#define CONFIG_SWAP_NONATOMIC 1
#define CONFIG_SYS_CLOCK_EXISTS 1
#define CONFIG_TIMEOUT_64BIT 1
#define CONFIG_SYS_CLOCK_MAX_TIMEOUT_DAYS 365
#define CONFIG_STACK_POINTER_RANDOM 0
#define CONFIG_ARCH_MEM_DOMAIN_SUPPORTS_ISOLATED_STACKS 1
#define CONFIG_MEM_DOMAIN_ISOLATED_STACKS 1
#define CONFIG_MP_NUM_CPUS 1
#define CONFIG_TOOLCHAIN_SUPPORTS_THREAD_LOCAL_STORAGE 1
#define CONFIG_THREAD_LOCAL_STORAGE 1
#define CONFIG_TOOLCHAIN_SUPPORTS_STATIC_INIT_GNU 1
#define CONFIG_KERNEL_INIT_PRIORITY_OBJECTS 30
#define CONFIG_KERNEL_INIT_PRIORITY_LIBC 35
#define CONFIG_KERNEL_INIT_PRIORITY_DEFAULT 40
#define CONFIG_KERNEL_INIT_PRIORITY_DEVICE 50
#define CONFIG_APPLICATION_INIT_PRIORITY 90
#define CONFIG_FLASH 1
#define CONFIG_CLOCK_CONTROL_NRF 1
#define CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL 1
#define CONFIG_CLOCK_CONTROL_NRF_K32SRC_50PPM 1
#define CONFIG_CLOCK_CONTROL_NRF_ACCURACY 50
#define CONFIG_CONSOLE_INPUT_MAX_LINE_LEN 128
#define CONFIG_CONSOLE_INIT_PRIORITY 40
#define CONFIG_ENTROPY_GENERATOR 1
#define CONFIG_ENTROPY_PSA_CRYPTO_RNG 1
#define CONFIG_ENTROPY_HAS_DRIVER 1
#define CONFIG_FLASH_HAS_DRIVER_ENABLED 1
#define CONFIG_FLASH_HAS_EXPLICIT_ERASE 1
#define CONFIG_FLASH_HAS_PAGE_LAYOUT 1
#define CONFIG_FLASH_PAGE_LAYOUT 1
#define CONFIG_SOC_FLASH_NRF 1
#define CONFIG_SOC_FLASH_NRF_RADIO_SYNC_NONE 1
#define CONFIG_GPIO_NPM1300 1
#define CONFIG_GPIO_NPM1300_INIT_PRIORITY 85
#define CONFIG_GPIO_NRFX 1
#define CONFIG_GPIO_NRFX_INTERRUPT 1
#define CONFIG_I2C_NRFX 1
#define CONFIG_I2C_NRFX_TWIM 1
#define CONFIG_I2C_NRFX_TRANSFER_TIMEOUT 500
#define CONFIG_I2C_INIT_PRIORITY 50
#define CONFIG_INTC_INIT_PRIORITY 40
#define CONFIG_MFD_INIT_PRIORITY 80
#define CONFIG_MFD_NPM1300 1
#define CONFIG_MFD_NPM1300_INIT_PRIORITY 80
#define CONFIG_PINCTRL_STORE_REG 1
#define CONFIG_PINCTRL_NRF 1
#define CONFIG_REGULATOR_THREAD_SAFE_REFCNT 1
#define CONFIG_REGULATOR_NPM1300 1
#define CONFIG_REGULATOR_NPM1300_COMMON_INIT_PRIORITY 85
#define CONFIG_REGULATOR_NPM1300_INIT_PRIORITY 86
#define CONFIG_SENSOR_INIT_PRIORITY 90
#define CONFIG_BME680 1
#define CONFIG_BME680_TEMP_OVER_2X 1
#define CONFIG_BME680_PRESS_OVER_16X 1
#define CONFIG_BME680_HUMIDITY_OVER_1X 1
#define CONFIG_BME680_FILTER_OFF 1
#define CONFIG_BME680_HEATR_TEMP_LP 1
#define CONFIG_BME680_HEATR_DUR_LP 1
#define CONFIG_NPM1300_CHARGER 1
#define CONFIG_SYSTEM_CLOCK_INIT_PRIORITY 0
#define CONFIG_TICKLESS_CAPABLE 1
#define CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT 1
#define CONFIG_SYSTEM_CLOCK_WAIT_FOR_STABILITY 1
#define CONFIG_NET_MGMT_EVENT_STACK_SIZE 2048
#define CONFIG_NET_TC_TX_COUNT 0
#define CONFIG_NET_INTERFACE_NAME_LEN 8
#define CONFIG_NET_MGMT_EVENT_QUEUE_SIZE 5
#define CONFIG_REQUIRES_FULL_LIBC 1
#define CONFIG_FULL_LIBC_SUPPORTED 1
#define CONFIG_MINIMAL_LIBC_SUPPORTED 1
#define CONFIG_NEWLIB_LIBC_SUPPORTED 1
#define CONFIG_PICOLIBC_SUPPORTED 1
#define CONFIG_PICOLIBC 1
#define CONFIG_HAS_NEWLIB_LIBC_NANO 1
#define CONFIG_COMMON_LIBC_ABORT 1
#define CONFIG_COMMON_LIBC_MALLOC 1
#define CONFIG_COMMON_LIBC_CALLOC 1
#define CONFIG_COMMON_LIBC_REALLOCARRAY 1
#define CONFIG_PICOLIBC_USE_TOOLCHAIN 1
#define CONFIG_PICOLIBC_IO_FLOAT 1
#define CONFIG_NEED_LIBC_MEM_PARTITION 1
#define CONFIG_CRC 1
#define CONFIG_SYS_HASH_FUNC32 1
#define CONFIG_SYS_HASH_FUNC32_MURMUR3 1
#define CONFIG_SYS_HASH_FUNC32_CHOICE_MURMUR3 1
#define CONFIG_SYS_HEAP_ALLOC_LOOPS 3
#define CONFIG_SYS_HEAP_AUTO 1
#define CONFIG_NET_BUF 1
#define CONFIG_NET_BUF_ALIGNMENT 0
#define CONFIG_FDTABLE 1
#define CONFIG_ZVFS_OPEN_MAX 4
#define CONFIG_HAS_POWEROFF 1
#define CONFIG_CBPRINTF_COMPLETE 1
#define CONFIG_CBPRINTF_FULL_INTEGRAL 1
#define CONFIG_CBPRINTF_FP_SUPPORT 1
#define CONFIG_CBPRINTF_LIBC_SUBSTS 1
#define CONFIG_CBPRINTF_CONVERT_CHECK_PTR 1
#define CONFIG_POSIX_AEP_CHOICE_NONE 1
#define CONFIG_POSIX_OPEN_MAX 4
#define CONFIG_POSIX_PAGE_SIZE 0x40
#define CONFIG_POSIX_TIMERS 1
#define CONFIG_POSIX_MONOTONIC_CLOCK 1
#define CONFIG_POSIX_CPUTIME 1
#define CONFIG_POSIX_CLOCK_SELECTION 1
#define CONFIG_POSIX_DELAYTIMER_MAX 32
#define CONFIG_POSIX_TIMER_MAX 32
#define CONFIG_POSIX_TIMEOUTS 1
#define CONFIG_TIMER_CREATE_WAIT 100
#define CONFIG_EVENTFD_MAX 0
#define CONFIG_MAX_PTHREAD_COUNT 0
#define CONFIG_MAX_PTHREAD_KEY_COUNT 0
#define CONFIG_MAX_TIMER_COUNT 32
#define CONFIG_MSG_COUNT_MAX 0
#define CONFIG_POSIX_CLOCK 1
#define CONFIG_POSIX_LIMITS_RTSIG_MAX 0
#define CONFIG_POSIX_MAX_FDS 4
#define CONFIG_POSIX_MAX_OPEN_FILES 4
#define CONFIG_TIMER_DELAYTIMER_MAX 32
#define CONFIG_SEM_NAMELEN_MAX 0
#define CONFIG_SEM_VALUE_MAX 0
#define CONFIG_TC_PROVIDES_POSIX_C_LANG_SUPPORT_R 1
#define CONFIG_LIBGCC_RTLIB 1
#define CONFIG_NOTIFY 1
#define CONFIG_ONOFF 1
#define CONFIG_PRINTK 1
#define CONFIG_EARLY_CONSOLE 1
#define CONFIG_ASSERT_LEVEL 2
#define CONFIG_SPIN_VALIDATE 1
#define CONFIG_ASSERT_VERBOSE 1
#define CONFIG_ASSERT_NO_COND_INFO 1
#define CONFIG_ASSERT_NO_MSG_INFO 1
#define CONFIG_IMG_MANAGER 1
#define CONFIG_MCUBOOT_IMG_MANAGER 1
#define CONFIG_MCUBOOT_TRAILER_SWAP_TYPE 1
#define CONFIG_MCUBOOT_UPDATE_FOOTER_SIZE 0x2000
#define CONFIG_IMG_BLOCK_BUF_SIZE 512
#define CONFIG_FCB 1
#define CONFIG_LLEXT_EDK_NAME "llext-edk"
#define CONFIG_MEM_ATTR 1
#define CONFIG_NETWORKING 1
#define CONFIG_NET_IP 1
#define CONFIG_NET_NATIVE 1
#define CONFIG_NET_NATIVE_IP 1
#define CONFIG_NET_NATIVE_IPV6 1
#define CONFIG_NET_NATIVE_IPV4 1
#define CONFIG_NET_NATIVE_UDP 1
#define CONFIG_NET_OFFLOADING_SUPPORT 1
#define CONFIG_NET_INIT_PRIO 90
#define CONFIG_NET_IP_DSCP_ECN 1
#define CONFIG_NET_IF_MAX_IPV6_COUNT 1
#define CONFIG_NET_IF_UNICAST_IPV6_ADDR_COUNT 2
#define CONFIG_NET_IF_MCAST_IPV6_ADDR_COUNT 3
#define CONFIG_NET_IF_IPV6_PREFIX_COUNT 2
#define CONFIG_NET_IPV6_MTU 1280
#define CONFIG_NET_INITIAL_HOP_LIMIT 64
#define CONFIG_NET_INITIAL_MCAST_HOP_LIMIT 1
#define CONFIG_NET_IPV6_MAX_NEIGHBORS 8
#define CONFIG_NET_IPV6_LOG_LEVEL 0
#define CONFIG_NET_IPV6_ND_LOG_LEVEL 0
#define CONFIG_NET_IPV6_PE_LOG_LEVEL 0
#define CONFIG_NET_ICMPV6_LOG_LEVEL 0
#define CONFIG_NET_IPV6_NBR_CACHE_LOG_LEVEL 0
#define CONFIG_NET_IPV4 1
#define CONFIG_NET_IF_MAX_IPV4_COUNT 1
#define CONFIG_NET_IF_UNICAST_IPV4_ADDR_COUNT 1
#define CONFIG_NET_IF_MCAST_IPV4_ADDR_COUNT 1
#define CONFIG_NET_INITIAL_TTL 64
#define CONFIG_NET_INITIAL_MCAST_TTL 1
#define CONFIG_NET_IF_MCAST_IPV4_SOURCE_COUNT 1
#define CONFIG_NET_IPV4_LOG_LEVEL 0
#define CONFIG_NET_ICMPV4_LOG_LEVEL 0
#define CONFIG_NET_IPV4_ACD_LOG_LEVEL 0
#define CONFIG_NET_TC_RX_COUNT 1
#define CONFIG_NET_TC_THREAD_COOPERATIVE 1
#define CONFIG_NET_TC_NUM_PRIORITIES 16
#define CONFIG_NET_TC_MAPPING_STRICT 1
#define CONFIG_NET_TX_DEFAULT_PRIORITY 1
#define CONFIG_NET_RX_DEFAULT_PRIORITY 0
#define CONFIG_NET_IP_ADDR_CHECK 1
#define CONFIG_NET_MAX_ROUTERS 2
#define CONFIG_NET_UDP 1
#define CONFIG_NET_UDP_MISSING_CHECKSUM 1
#define CONFIG_NET_UDP_LOG_LEVEL 0
#define CONFIG_NET_MAX_CONN 8
#define CONFIG_NET_MAX_CONTEXTS 6
#define CONFIG_NET_CONTEXT_SYNC_RECV 1
#define CONFIG_NET_CONTEXT_CHECK 1
#define CONFIG_NET_CONTEXT_DSCP_ECN 1
#define CONFIG_NET_CONTEXT_REUSEADDR 1
#define CONFIG_NET_CONTEXT_REUSEPORT 1
#define CONFIG_NET_BUF_RX_COUNT 16
#define CONFIG_NET_BUF_TX_COUNT 16
#define CONFIG_NET_BUF_FIXED_DATA_SIZE 1
#define CONFIG_NET_BUF_DATA_SIZE 128
#define CONFIG_NET_PKT_BUF_USER_DATA_SIZE 4
#define CONFIG_NET_DEFAULT_IF_FIRST 1
#define CONFIG_NET_INTERFACE_NAME 1
#define CONFIG_NET_MGMT 1
#define CONFIG_NET_MGMT_EVENT 1
#define CONFIG_NET_MGMT_EVENT_THREAD 1
#define CONFIG_NET_MGMT_EVENT_QUEUE 1
#define CONFIG_NET_MGMT_EVENT_QUEUE_TIMEOUT 10
#define CONFIG_NET_MGMT_EVENT_INFO 1
#define CONFIG_NET_MGMT_EVENT_LOG_LEVEL 0
#define CONFIG_NET_PKT_LOG_LEVEL 0
#define CONFIG_NET_DEBUG_NET_PKT_EXTERNALS 0
#define CONFIG_NET_CORE_LOG_LEVEL 0
#define CONFIG_NET_IF_LOG_LEVEL 0
#define CONFIG_NET_TC_LOG_LEVEL 0
#define CONFIG_NET_UTILS_LOG_LEVEL 0
#define CONFIG_NET_CONTEXT_LOG_LEVEL 0
#define CONFIG_NET_CONN_LOG_LEVEL 0
#define CONFIG_NET_ROUTE_LOG_LEVEL 0
#define CONFIG_MQTT_LIB 1
#define CONFIG_MQTT_LOG_LEVEL 0
#define CONFIG_MQTT_KEEPALIVE 120
#define CONFIG_MQTT_LIB_TLS 1
#define CONFIG_NET_HTTP_LOG_LEVEL 0
#define CONFIG_NET_HTTP_SERVER_LOG_LEVEL 0
#define CONFIG_SNTP 1
#define CONFIG_SNTP_LOG_LEVEL 0
#define CONFIG_NET_CONFIG_LOG_LEVEL 0
#define CONFIG_NET_SOCKETS 1
#define CONFIG_NET_SOCKETS_PRIORITY_DEFAULT 50
#define CONFIG_NET_SOCKETS_POSIX_NAMES 1
#define CONFIG_NET_SOCKETS_POLL_MAX 3
#define CONFIG_NET_SOCKETS_CONNECT_TIMEOUT 3000
#define CONFIG_NET_SOCKET_MAX_SEND_WAIT 10000
#define CONFIG_NET_SOCKETS_TLS_PRIORITY 45
#define CONFIG_NET_SOCKETS_TLS_SET_MAX_FRAGMENT_LENGTH 1
#define CONFIG_NET_SOCKETS_OFFLOAD 1
#define CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY 40
#define CONFIG_NET_SOCKETS_LOG_LEVEL 0
#define CONFIG_NET_DHCPV6_DUID_MAX_LEN 22
#define CONFIG_NET_CONNECTION_MANAGER 1
#define CONFIG_NET_CONNECTION_MANAGER_LOG_LEVEL 0
#define CONFIG_NET_CONNECTION_MANAGER_MONITOR_PRIORITY 1
#define CONFIG_NET_CONNECTION_MANAGER_AUTO_IF_DOWN 1
#define CONFIG_TIMER_RANDOM_INITIAL_STATE 123456789
#define CONFIG_ENTROPY_DEVICE_RANDOM_GENERATOR 1
#define CONFIG_CSPRNG_ENABLED 1
#define CONFIG_HARDWARE_DEVICE_CS_GENERATOR 1
#define CONFIG_SETTINGS 1
#define CONFIG_SETTINGS_DYNAMIC_HANDLERS 1
#define CONFIG_SETTINGS_FCB 1
#define CONFIG_SETTINGS_FCB_NUM_AREAS 8
#define CONFIG_SETTINGS_FCB_MAGIC 0xc0ffeeee
#define CONFIG_FLASH_MAP 1
#define CONFIG_STREAM_FLASH 1
#define CONFIG_STREAM_FLASH_ERASE 1
#define CONFIG_TOOLCHAIN_ZEPHYR_0_17 1
#define CONFIG_TOOLCHAIN_ZEPHYR_SUPPORTS_THREAD_LOCAL_STORAGE 1
#define CONFIG_TOOLCHAIN_ZEPHYR_SUPPORTS_GNU_EXTENSIONS 1
#define CONFIG_LINKER_ORPHAN_SECTION_WARN 1
#define CONFIG_ROM_END_OFFSET 0x2000
#define CONFIG_LD_LINKER_SCRIPT_SUPPORTED 1
#define CONFIG_LD_LINKER_TEMPLATE 1
#define CONFIG_LINKER_SORT_BY_ALIGNMENT 1
#define CONFIG_LINKER_GENERIC_SECTIONS_PRESENT_AT_BOOT 1
#define CONFIG_LINKER_LAST_SECTION_ID 1
#define CONFIG_LINKER_LAST_SECTION_ID_PATTERN 0xE015E015
#define CONFIG_LINKER_USE_RELAX 1
#define CONFIG_LINKER_ITERABLE_SUBALIGN 4
#define CONFIG_LINKER_DEVNULL_SUPPORT 1
#define CONFIG_STD_C99 1
#define CONFIG_TOOLCHAIN_SUPPORTS_GNU_EXTENSIONS 1
#define CONFIG_SIZE_OPTIMIZATIONS 1
#define CONFIG_COMPILER_TRACK_MACRO_EXPANSION 1
#define CONFIG_COMPILER_COLOR_DIAGNOSTICS 1
#define CONFIG_FORTIFY_SOURCE_COMPILE_TIME 1
#define CONFIG_COMPILER_OPT ""
#define CONFIG_RUNTIME_ERROR_CHECKS 1
#define CONFIG_KERNEL_BIN_NAME "zephyr"
#define CONFIG_OUTPUT_STAT 1
#define CONFIG_OUTPUT_PRINT_MEMORY_USAGE 1
#define CONFIG_BUILD_GAP_FILL_PATTERN 0xFF
#define CONFIG_BUILD_OUTPUT_STRIP_PATHS 1
#define CONFIG_CHECK_INIT_PRIORITIES 1
#define CONFIG_DEPRECATED 1
#define CONFIG_WARN_DEPRECATED 1
#define CONFIG_EXPERIMENTAL 1
#define CONFIG_ENFORCE_ZEPHYR_STDINT 1
#define CONFIG_LEGACY_GENERATED_INCLUDE_PATH 1
