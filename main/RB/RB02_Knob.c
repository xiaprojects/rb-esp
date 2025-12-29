
#include "RB02_NavCore.h"

#ifdef RB02_BUTTON_KNOB
#include "RB02_Navigator.h"

// TODO: be moved into shared structure
extern lv_obj_t *mbox1;
extern lv_obj_t *tv;
// TODO: Move the RB02.c code into RB02_Navigator.c
extern uint8_t DeviceIsDemoMode;
void nvsStoreDefaultScreenOrDemo();
void actionInTab(touchLocation location);

void RB02_NavCore_InjectTouch(uint8_t touch_loc)
{
  // Same protections as speedBgClicked
  if (mbox1 != NULL)
    return;

  if (tv == NULL)
    return;

  bool changedTab = false;
  lv_tabview_t *tabview = (lv_tabview_t *)tv;
  uint16_t cur = tabview->tab_cur;

  // Reproduce the "global" W/E tab switching behavior from speedBgClicked()
  switch (touch_loc)
  {
    case RB02_TOUCHLOC_W:
      if (cur > 0)
      {
        cur--;
        changedTab = true;
      }
      break;

    case RB02_TOUCHLOC_E:
      cur++;
      changedTab = true;
      if (cur >= RB02_TAB_DEV)
      {
        cur = RB02_TAB_SET;
      }
      break;

    default:
      // For N/S/CENTER etc, reuse RB02 dispatching
      actionInTab((touchLocation)touch_loc);
      break;
  }

  if (changedTab)
  {
    lv_tabview_set_act(tv, cur, LV_ANIM_OFF);

    if (DeviceIsDemoMode == 0)
    {
      nvsStoreDefaultScreenOrDemo();
    }
  }
}
#endif
