

void udp_callback(uint8_t *data, uint8_t datalen, uint8_t *recv, uint8_t *reclen)
{
    
   // printf("%d %s\n",datalen,data);
     
            package_t pk = {};
            pk.package = atol((char *)data);

            backword_t rt = {};
            dali_callback(&pk, &rt);


                if (rt.backword!=BACK_NONE)
                {
                    uint8_t dat = 0x00;
                    if (rt.backword==BACK_DATA) dat=rt.data; else dat=rt.backword;
                    sprintf((char*)recv,"%d",dat);
                    *reclen=strlen((char*)recv);

                } 
         

            /*
            address_t adres = {};
            unpackage_address(pk.data.data0,&adres);
           
                    printf("arc power %d\n", adres.arc_power);
                    printf("short adr %d\n", adres.short_adr);
                    printf("group adr %d\n", adres.group_adr);
                    printf("broadcast %d\n", adres.broadcast_adr);
                    printf("special %d\n", adres.special);
                    printf("data %02X\n", adres.data);
                    printf("error %d\n", adres.error);
                    printf("COMMAND %02X %02X %d\n", pk.data.data0,pk.data.data1,pk.data.type);  
           
            if (!adres.error)
            {    
                                    
            }
            */

}

