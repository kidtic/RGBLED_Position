#include"ScanRelocate.h"


namespace LED_POSITION
{

ScanRelocate::ScanRelocate(System *pSys):mpSystem(pSys)
{
}

ScanRelocate::~ScanRelocate()
{
}

void ScanRelocate::Run()
{
    while (1)
    {
        sleep(1);
        cout<<"ScanRelocate thread"<<endl;
    }
    
}

}

