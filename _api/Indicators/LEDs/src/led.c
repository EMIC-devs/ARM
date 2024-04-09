#include "inc/led_.{name}..h"

void LEDs_.{name}._init (void)
{
	HAL_GPIO_PinCfg(.{pin}., GPIO_OUTPUT);
}

EMIC:ifdef usedFunction.LEDs_.{name}._state
void LEDs_.{name}._state(uint8_t status)
{
	switch (status)
	{
		case 0:
			HAL_GPIO_PinSet(.{pin}., GPIO_LOW); // es mal
			break;
		case 1:
			HAL_GPIO_PinSet(.{pin}., GPIO_HIGH);
			break;
		case 2:
			if (HAL_GPIO_PinGet(.{pin}.))
            {
				HAL_GPIO_PinSet(.{pin}.,GPIO_LOW);
			}
			else 
			{
				HAL_GPIO_PinSet(.{pin}.,GPIO_HIGH);
			}
			break;
	}
}
EMIC:endif

EMIC:ifdef usedFunction.LEDs_.{name}._blink
void LEDs_.{name}._poll ()
{

}
EMIC:endif	
