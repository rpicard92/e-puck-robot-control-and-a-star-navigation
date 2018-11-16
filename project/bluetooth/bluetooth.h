/*
 * This package contains all the ressources you need to control the bluetooth
 * device you have on your e-puck AS MASTER. If you only want to use the bluetooth
 * AS SLAVE, the uart library is enough (the connection to a master look like a 
 * standard uart connection).
 * \n The bluetooth device is connected on the uart1.
 * \n \n Generally, the bluetooth modules have a bluetooth class device which identify
 * them as PC, mobile, mouse, ... The e-puck bluetooth module has the default
 * device class number, it's 000.
 * \n \n To learn more about the e-puck's bluetooth device see the documentation of the
 * LMX9820A from National Semiconductor (look at
 * http://www.national.com/mpf/LM/LMX9820A.html) and look at the \ref uart module.
 *
 */

#ifndef _BLUETOOTH
#define _BLUETOOTH

/*! \struct BtDevice
 * \brief general struct for bluetooth device
 */
struct BtDevice
{
  unsigned char address[6]; /*!< Bluetooth MAC address */
  unsigned char class[3];	/*!< Bluetooth class of device*/
  char friendly_name[20];	/*!< The friendly name of the bluetooth device */
};

/*! \struct BtEPuck
 * \brief general struct for other e-puck
 */
struct BtEPuck
{
  unsigned char address[6];	/*!< e-puck's bluetooth MAC address */
  unsigned char number[5];	/*!< e-puck's bluetooth PIN */
};

/*global variable can be access to be read*/
extern unsigned char e_bt_local_paired_device[6*8];
extern struct BtDevice e_bt_present_device[10];		/*!< An extern array containing all the bluetooth device detected. It's carried out by the fonction e_bt_find_epuck
													* \sa e_bt_find_epuck */
extern struct BtEPuck e_bt_present_epuck[10];		/*!< An extern array containing all the e-puck detected. It's carried out by the fonction e_bt_find_epuck
													* \sa e_bt_find_epuck */

/*------ special e-puck fuction ------*/

int e_bt_find_epuck(void);
char e_bt_connect_epuck(void);


/*------ low level bluettoth fuction ------*/

char e_bt_read_local_pin_number(char *PIN);
char e_bt_read_local_name(char *name); 

char e_bt_write_local_pin_number(char *PIN); //give 1 to 16 number PIN (in general e-puck have 4 numbers PIN)
char e_bt_write_local_name(char *name);//write friendly name

char e_bt_etablish_SPP_link(char *address);//established connection to BT address given
char e_bt_release_SPP_link(void);
char e_bt_send_SPP_data(char *data,char datalenght);//send data maximum 127 bytes, if you are in non transparent mode
char e_bt_tranparent_mode(void);//pass in transparent mode
void e_bt_exit_tranparent_mode(void);//generate break

char e_bt_list_local_paired_device(void);
char e_bt_remove_local_paired_device(int);

int e_bt_inquiry(struct BtDevice *device);//return number of device found
char e_bt_get_friendly_name(struct BtDevice *device);

char e_bt_set_event_filter(char event);//set the event filter mode 
char e_bt_get_event_filter(void);


int e_bt_reset(void);
char  e_bt_factory_reset(void);//WARNING use this function only if you are sure, your e-puck must be than restart, and renamed!!!

char e_bt_recv_SPP_data(char *buffer);
char e_bt_set_scanmode(char connectability, char discoverability);
void e_bt_exit_transparent_mode(void); //Generate UART break.
void e_bt_restore_pin(void);

#endif
