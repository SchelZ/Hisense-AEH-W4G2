# Hisense AC flashing with esphome
# ! This project was done for Hisense Wings Comfort 12000BTU, code AERKB35MR0B
* Wi-Fi module used in this project is AEH-W4G2
* This project scopes in flashing esphome on AEH-W4G2 ; have all the commands that Hisense app has ; have OTA available for easier features updates
* RAM: 14.0 % (used 36584 bytes from 262144 bytes)
* Flash: 93.6% (used 448652 bytes from 479232 bytes)
</br>

### ! This is the circuit diagram used to sniff the commands made by AEH-W4G2 Wi-Fi module using ESP32 !
<img width="1566" height="1818" alt="circuit_image" src="https://github.com/user-attachments/assets/f10cdf1b-b5d6-475b-89b7-221ddf9a1ca4" />

</br>
</br>

### ! This is the circuit diagram used for flashing AEH-W4G2 Wi-Fi module !
* I found out that using 3v3 on the ttl adaptor is getting AEH-W4G2 to enter flashing mode, and 5V is only for powering
* You also need to have AEH-W4G2 TX pin wired to GND the moment you plug the TTL adaptor so is entering flash mode and then you have to remove AEH-W4G2 TX pin from GND and connect it to TX pin of the TTL adaptor
<img width="1987" height="950" alt="circuit_image (1)" src="https://github.com/user-attachments/assets/5142624c-9454-4600-bd3d-05690177afc5" />


