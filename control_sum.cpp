
#include <iostream> // standart I/O facilities included
#include <string>
#include<algorithm>
#include<vector>
#include<cmath>
#include<stdio.h>
using namespace std;

#define TX_LEN 60

uint32_t xor_checksum(uint32_t *data, size_t len)
{
    uint32_t t = 0;
    for (int i = 0; i < len; i++)
    {
        t = t ^ data[i];
        printf("%x\n",t);
    }
    return t;
}

uint8_t tx_buff[TX_LEN] = {0};

int main() {

    for (int i = 0; i < TX_LEN; i++) {
        tx_buff[i] = i+1;
    }
    uint32_t calc_checksum = xor_checksum((uint32_t *)tx_buff, 14);
    cout << calc_checksum << "\n";

}