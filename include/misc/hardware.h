enum SKU_INFO {
	SKU_LTE_ROW  = 0,
	SKU_LTE_PRC  = 1,
	SKU_WIFI_ROW = 2,
	SKU_WIFI_PRC = 3,
	SKU_INVALID
};

enum HW_INFO {
	HW_EVB = 0,
	HW_EVT = 1,
	HW_DVT1 = 2,
	HW_DVT2 = 3,
	HW_PVT = 4,
	HW_INVALID
};

enum HW_INFO hwid_get_stage(void);

enum SKU_INFO hwid_get_sku(void);
