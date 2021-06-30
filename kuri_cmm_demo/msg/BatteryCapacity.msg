# DO NOT MODIFY -- direct copy of kuri message
int16 mAh
int8 pct
int8 rounded_pct # capacity percentage estimate, rounded to the nearest 5 and
                 # subjected to hysteresis so it can only change if the value has
                 # changed by at least 5
