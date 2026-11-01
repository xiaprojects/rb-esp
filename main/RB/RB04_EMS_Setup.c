#ifdef RB_ENABLE_EMS

void nvsStoreEMS(uint8_t isFarenheit)
{
  //
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    // Read
    err = nvs_set_u8(my_handle, "isFarenheit", isFarenheit);
    nvs_close(my_handle);
  }
}

static void RB04_Setup_Changed_Farenheit(lv_event_t *e)
{
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_user_data(e);
  if (lv_obj_get_state(sw) & LV_STATE_CHECKED)
  {
    singletonConfig()->ui.displayFarenheit = 1;
  }
  else
  {
    singletonConfig()->ui.displayFarenheit = 0;
  }
    nvsStoreEMS(singletonConfig()->ui.displayFarenheit);
}

lv_obj_t *RB02_Setup_CreateScreen_EMS(RB02_Status *status, lv_obj_t *parent, int *lineY){
  if (true)
  {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_size(label, 150, 16);
    lv_obj_align(label, LV_ALIGN_CENTER, -75, *lineY);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);
    lv_label_set_text(label, "Farenheit Scale");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_t *sw = lv_switch_create(parent);
    if (status->ui.displayFarenheit != 0)
    {
      lv_obj_add_state(sw, LV_STATE_CHECKED);
    }
    lv_obj_set_size(sw, 65, 40);
    lv_obj_align(sw, LV_ALIGN_CENTER, 40, *lineY);
    lv_obj_add_event_cb(sw, RB04_Setup_Changed_Farenheit, LV_EVENT_VALUE_CHANGED, sw);
    *lineY += 40;
  }
  return NULL;
}
#endif
