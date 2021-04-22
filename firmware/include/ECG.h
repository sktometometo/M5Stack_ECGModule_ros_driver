#include <M5Stack.h>

enum ECGState{
    ECG_IDLE,
    ECG_DATA_LENGTH,
    ECG_DATA_H,
    ECG_DATA_L,
    ECG_END,
};

class ECG {
public:
    ECG();

    void readData();
    void display(TFT_eSprite& tft_chart, TFT_eSprite& tft_num);
    uint16_t getData(int index);

private:
    uint8_t rxBuff[1024];
    uint8_t SendBuff[7] = { 0xAA, 0x55, 
                            0x03, //length
                            0x00, 0x00,
                            0x00,
                            0xef};
    uint8_t ECGdataFSM = ECG_IDLE;
    uint8_t ECGDataLength = 0;
    uint8_t dataCount = 0;
    uint8_t flushFlag = 0;
    uint16_t ECGData16Buff[128];

    uint16_t    ECGDataBuff[640];
    uint16_t    ECG16Bitdata;
    int16_t     ECGWritePoint = 320;
    int16_t     ECGReadPoint = 0;

    uint16_t    ECGLastDataY = 0;
};

ECG::ECG()
{
    Serial2.begin(115200,SERIAL_8N1,13,5);
}

void ECG::readData()
{
    uint8_t data;
    if( Serial2.available())
    {
        size_t size = Serial2.available();
        Serial2.readBytes(rxBuff,size);

        for( int i = 0; i < size; i++ )
        {
            data = rxBuff[i];
            switch( ECGdataFSM )
            {
                case ECG_IDLE :
                    ECGdataFSM = ( data == 0xAA ) ? ECG_DATA_LENGTH : ECG_IDLE;
                break;
                case ECG_DATA_LENGTH :
                    ECGDataLength = data;
                    dataCount = 0;
                    ECGdataFSM = ECG_DATA_H;
                break;
                case ECG_DATA_H : 
                    ECG16Bitdata = 0;
                    ECG16Bitdata = data;
                    ECG16Bitdata <<= 8;
                    ECGdataFSM = ECG_DATA_L;
                break;
                case ECG_DATA_L :
                    ECG16Bitdata |= data;
                    ECGData16Buff[dataCount] = ECG16Bitdata;
                    dataCount ++;
                    ECGdataFSM = ( dataCount >= ECGDataLength ) ? ECG_END : ECG_DATA_H;
                break;
                case ECG_END:
                    if( data == 0xef )
                    {
                        ECGDataBuff[( ECGWritePoint % 640 )] = ECGData16Buff[0];
                        ECGWritePoint = ( ECGWritePoint + 1 ) % 640;
                        ECGReadPoint =  ( ECGReadPoint + 1  ) % 640;
                        flushFlag = 1;
                    }
                    ECGdataFSM = ECG_IDLE;
                break;
            }
        }
    }
}

void ECG::display(TFT_eSprite& tft_chart, TFT_eSprite& tft_num)
{
    uint16_t ECGData;
    int16_t  DataMAX, DataMIN;

    if( flushFlag != 1 ) return;

    flushFlag = 0;

    tft_chart.fillRect(0,0,320,100,BLACK);

    for( size_t i = 0; i < 320; i++ )
    {
        ECGData = ECGDataBuff[ ( ECGReadPoint + i ) % 640 ] ;
        if( i == 0 )
        {
            DataMAX = ECGData;
            DataMIN = ECGData;
        }
        else
        {
            DataMAX = ( ECGData > DataMAX ) ? ECGData : DataMAX;
            DataMIN = ( ECGData < DataMIN ) ? ECGData : DataMIN;
        }
    }
    DataMAX = DataMAX + ( DataMAX / 10 );
    DataMIN = DataMIN - ( DataMIN / 10 );
    DataMIN = ( DataMIN < 0 ) ? 0 : DataMIN;

    for( size_t i = 0; i < 320; i++ )
    {
        ECGData = ECGDataBuff[ ( ECGReadPoint + i ) % 640 ];
        ECGData = map( ECGData, DataMIN, DataMAX, 100, 0 );
        tft_chart.drawLine(i, ECGLastDataY, i+1, ECGData, RED);
        ECGLastDataY = ECGData;
    }
    tft_chart.pushSprite(0,100);

    if( ECGData16Buff[3] & 0x0100 )
    {
        tft_num.fillRect(0,0,150,100,BLACK);
        tft_num.fillRect(0,0,20,100,GREEN);

        tft_num.setTextFont(8);
        tft_num.setTextColor(WHITE);
        tft_num.drawNumber( ECGData16Buff[3]&0x00ff, 30, 14 );
    }
    else
    {
        tft_num.fillRect(0,0,150,100,BLACK);
        tft_num.fillRect(0,0,20,100,RED);
    }
    tft_num.pushSprite(0,0);
}

uint16_t ECG::getData(int index)
{
    return ECGDataBuff[ ( ECGReadPoint + index ) % 640 ];
}
