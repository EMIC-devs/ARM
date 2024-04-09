void LEDs_.{name}._init (void);
EMIC:define(inits.LEDs_.{name}._init,LEDs_.{name}._init)

void LEDs_.{name}._poll (void);
EMIC:define(polls.LEDs_.{name}._poll,LEDs_.{name}._poll)


EMIC:ifdef usedFunction.LEDs_.{name}._state

void LEDs_.{name}._state(uint8_t);

EMIC:endif