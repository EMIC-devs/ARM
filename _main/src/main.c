
#include "main.h"


int main(void)
{
	EMIC:ifdef usedEvent.SystemConfig
	SystemConfig();
	EMIC:endif
	//se inicializan drivers

	.{inits.*}.();

	EMIC:ifdef usedEvent.onReset
	onReset();
	EMIC:endif

	while (1)
	{
		.{polls.*}.();
	}
}
