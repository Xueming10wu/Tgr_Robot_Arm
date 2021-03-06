#include "YodaEncoder.h"

int main(int argc, char const *argv[])
{
    //uint8_t a[] = {0x02, 0x03, 0x10, 0x00, 0x04, 0xf8, 0x7b, 0x00, 0x00,  0xf2, 0xfd };
    
    //uint8_t a[] = {0x02, 0x03, 0x10, 0x00, 0x04, 0xa9, 0x5b, 0x00, 0x01, 0x22, 0x0b};

    uint8_t a[] = {0x02, 0x03, 0x10, 0x00, 0x04, 0xa9, 0x57, 0x00, 0x01, 0xe2, 0x08};
     
    //int c = CRC16_MODBUS(a,9);

    int c = getEncoderAngle(a);
    if (c == -1)
    {
        cout << "校验失败" << endl;
    }
    else
    {
        cout << "数值为" << hex << c << ", " << dec << c << endl;
    }
    return 0;
}
