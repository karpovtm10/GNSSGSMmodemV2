#include "base64.h"
#include <string.h>

static const unsigned char base64_enc_map[64] =
{
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd',
    'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x',
    'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', '+', '/'
};
void base64_encode(char *str, char *enc_str)
{
	uint8_t i = 0;
	uint8_t c1, c2, c3;
	
	for (i = 0; i < 100;)
	{
		c1 = *str++;
		c2 = *str++;
		c3 = *str++;
		if (c1 == 0) break;
		enc_str[i++] = base64_enc_map[(c1 >> 2) & 0x3F];
		enc_str[i++] = base64_enc_map[(((c1 &  3) << 4) + (c2 >> 4)) & 0x3F];
		if (c2 == 0) break;
		enc_str[i++] = base64_enc_map[(((c2 & 15) << 2) + (c3 >> 6)) & 0x3F];
		if (c3 == 0) break;
		enc_str[i++] = base64_enc_map[c3 & 0x3F];
	}
	
	while (i % 4 != 0)
	{
		enc_str[i] = '=';
		i++;
	} 
	

}
