/*
use (&) before register
Example :
_SET(&PORTB,1,1);

_READ(&PINB,1);

-Arif Bryan
*/

#ifndef _BYTECON_H
#define _BYTECON_H

#define _READ(byte,bit)		((*byte>>(bit))&1)
#define _SET(byte,bit,st)	*byte = ((st) ? *byte|(1<<(bit)) : *byte&(~(1<<(bit))))

#endif