var searchIndex = {};
searchIndex["i2c_linux_sys"] = {"doc":"","items":[[3,"Flags","i2c_linux_sys","",null,null],[3,"Functionality","","To determine what functionality is present",null,null],[3,"i2c_msg","","an I2C transaction segment beginning with START",null,null],[12,"addr","","Slave address, either seven or ten bits.",0,null],[12,"flags","","`I2C_M_RD` is handled by all adapters.",0,null],[12,"len","","Number of data bytes in `buf` being read from or written to the I2C slave address.",0,null],[12,"buf","","The buffer into which data is read, or from which it's written.",0,null],[3,"i2c_rdwr_ioctl_data","","This is the structure as used in the I2C_RDWR ioctl call",null,null],[12,"msgs","","ptr to array of simple messages",1,null],[12,"nmsgs","","number of messages to exchange",1,null],[3,"i2c_smbus_data","","Data for SMBus Messages",null,null],[12,"block","","block[0] is used for length and one more for user-space compatibility",2,null],[3,"i2c_smbus_ioctl_data","","This is the structure as used in the I2C_SMBUS ioctl call",null,null],[12,"read_write","","",3,null],[12,"command","","",3,null],[12,"size","","",3,null],[12,"data","","",3,null],[4,"SmbusReadWrite","","i2c_smbus_xfer read or write markers",null,null],[13,"Read","","",4,null],[13,"Write","","",4,null],[4,"SmbusTransaction","","SMBus transaction types (size parameter in the above functions)",null,null],[13,"Quick","","",5,null],[13,"Byte","","",5,null],[13,"ByteData","","",5,null],[13,"WordData","","",5,null],[13,"ProcCall","","",5,null],[13,"BlockData","","",5,null],[13,"I2cBlockBroken","","",5,null],[13,"BlockProcCall","","SMBus 2.0",5,null],[13,"I2cBlockData","","",5,null],[5,"i2c_set_retries","","`I2C_RETRIES`",null,{"inputs":[{"name":"rawfd"},{"name":"usize"}],"output":{"name":"result"}}],[5,"i2c_set_timeout_ms","","`I2C_TIMEOUT`",null,{"inputs":[{"name":"rawfd"},{"name":"usize"}],"output":{"name":"result"}}],[5,"i2c_set_slave_address","","`I2C_SLAVE` and `I2C_SLAVE_FORCE`",null,{"inputs":[{"name":"rawfd"},{"name":"u16"},{"name":"bool"}],"output":{"name":"result"}}],[5,"i2c_set_slave_address_10bit","","`I2C_TENBIT`",null,{"inputs":[{"name":"rawfd"},{"name":"bool"}],"output":{"name":"result"}}],[5,"i2c_check_functionality","","`I2C_FUNCS`",null,{"inputs":[{"name":"rawfd"}],"output":{"generics":["functionality"],"name":"result"}}],[5,"i2c_pec","","`I2C_PEC`",null,{"inputs":[{"name":"rawfd"},{"name":"bool"}],"output":{"name":"result"}}],[5,"i2c_smbus","","`I2C_SMBUS`",null,{"inputs":[{"name":"rawfd"},{"name":"i2c_smbus_ioctl_data"}],"output":{"name":"result"}}],[5,"i2c_smbus_write_quick","","",null,{"inputs":[{"name":"rawfd"},{"name":"smbusreadwrite"}],"output":{"name":"result"}}],[5,"i2c_smbus_read_byte","","",null,{"inputs":[{"name":"rawfd"}],"output":{"generics":["u8"],"name":"result"}}],[5,"i2c_smbus_write_byte","","",null,{"inputs":[{"name":"rawfd"},{"name":"u8"}],"output":{"name":"result"}}],[5,"i2c_smbus_read_byte_data","","",null,{"inputs":[{"name":"rawfd"},{"name":"u8"}],"output":{"generics":["u8"],"name":"result"}}],[5,"i2c_smbus_write_byte_data","","",null,{"inputs":[{"name":"rawfd"},{"name":"u8"},{"name":"u8"}],"output":{"name":"result"}}],[5,"i2c_smbus_read_word_data","","",null,{"inputs":[{"name":"rawfd"},{"name":"u8"}],"output":{"generics":["u16"],"name":"result"}}],[5,"i2c_smbus_write_word_data","","",null,{"inputs":[{"name":"rawfd"},{"name":"u8"},{"name":"u16"}],"output":{"name":"result"}}],[5,"i2c_smbus_process_call","","",null,{"inputs":[{"name":"rawfd"},{"name":"u8"},{"name":"u16"}],"output":{"generics":["u16"],"name":"result"}}],[5,"i2c_smbus_read_block_data","","",null,null],[5,"i2c_smbus_write_block_data","","",null,null],[5,"i2c_smbus_read_i2c_block_data","","",null,null],[5,"i2c_smbus_write_i2c_block_data","","",null,null],[5,"i2c_smbus_block_process_call","","",null,null],[5,"i2c_rdwr","","",null,null],[0,"ioctls","","",null,null],[5,"i2c_retries","i2c_linux_sys::ioctls","",null,{"inputs":[{"name":"c_int"},{"name":"c_int"}],"output":{"generics":["c_int"],"name":"result"}}],[5,"i2c_timeout","","",null,{"inputs":[{"name":"c_int"},{"name":"c_int"}],"output":{"generics":["c_int"],"name":"result"}}],[5,"i2c_slave","","",null,{"inputs":[{"name":"c_int"},{"name":"c_int"}],"output":{"generics":["c_int"],"name":"result"}}],[5,"i2c_slave_force","","",null,{"inputs":[{"name":"c_int"},{"name":"c_int"}],"output":{"generics":["c_int"],"name":"result"}}],[5,"i2c_tenbit","","",null,{"inputs":[{"name":"c_int"},{"name":"c_int"}],"output":{"generics":["c_int"],"name":"result"}}],[5,"i2c_funcs","","",null,null],[5,"i2c_rdwr","","",null,null],[5,"i2c_pec","","",null,{"inputs":[{"name":"c_int"},{"name":"c_int"}],"output":{"generics":["c_int"],"name":"result"}}],[5,"i2c_smbus","","",null,null],[17,"I2C_M_RD","i2c_linux_sys","read data, from slave to master",null,null],[17,"I2C_M_TEN","","this is a ten bit chip address",null,null],[17,"I2C_M_RECV_LEN","","length will be first received byte",null,null],[17,"I2C_M_NO_RD_ACK","","if I2C_FUNC_PROTOCOL_MANGLING",null,null],[17,"I2C_M_IGNORE_NAK","","if I2C_FUNC_PROTOCOL_MANGLING",null,null],[17,"I2C_M_REV_DIR_ADDR","","if I2C_FUNC_PROTOCOL_MANGLING",null,null],[17,"I2C_M_NOSTART","","I2C_FUNC_NOSTART",null,null],[17,"I2C_M_STOP","","if I2C_FUNC_PROTOCOL_MANGLING",null,null],[17,"I2C_FUNC_I2C","","",null,null],[17,"I2C_FUNC_10BIT_ADDR","","",null,null],[17,"I2C_FUNC_PROTOCOL_MANGLING","","I2C_M_IGNORE_NAK etc.",null,null],[17,"I2C_FUNC_SMBUS_PEC","","",null,null],[17,"I2C_FUNC_NOSTART","","I2C_M_NOSTART",null,null],[17,"I2C_FUNC_SLAVE","","",null,null],[17,"I2C_FUNC_SMBUS_BLOCK_PROC_CALL","","SMBus 2.0",null,null],[17,"I2C_FUNC_SMBUS_QUICK","","",null,null],[17,"I2C_FUNC_SMBUS_READ_BYTE","","",null,null],[17,"I2C_FUNC_SMBUS_WRITE_BYTE","","",null,null],[17,"I2C_FUNC_SMBUS_READ_BYTE_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_WRITE_BYTE_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_READ_WORD_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_WRITE_WORD_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_PROC_CALL","","",null,null],[17,"I2C_FUNC_SMBUS_READ_BLOCK_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_WRITE_BLOCK_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_READ_I2C_BLOCK","","I2C-like block xfer",null,null],[17,"I2C_FUNC_SMBUS_WRITE_I2C_BLOCK","","w/ 1-byte reg. addr.",null,null],[17,"I2C_FUNC_SMBUS_HOST_NOTIFY","","",null,null],[17,"I2C_FUNC_SMBUS_BYTE","","",null,null],[17,"I2C_FUNC_SMBUS_BYTE_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_WORD_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_BLOCK_DATA","","",null,null],[17,"I2C_FUNC_SMBUS_I2C_BLOCK","","",null,null],[17,"I2C_FUNC_SMBUS_EMUL","","",null,null],[17,"I2C_SMBUS_READ","","",null,null],[17,"I2C_SMBUS_WRITE","","",null,null],[17,"I2C_SMBUS_QUICK","","",null,null],[17,"I2C_SMBUS_BYTE","","",null,null],[17,"I2C_SMBUS_BYTE_DATA","","",null,null],[17,"I2C_SMBUS_WORD_DATA","","",null,null],[17,"I2C_SMBUS_PROC_CALL","","",null,null],[17,"I2C_SMBUS_BLOCK_DATA","","",null,null],[17,"I2C_SMBUS_I2C_BLOCK_BROKEN","","",null,null],[17,"I2C_SMBUS_BLOCK_PROC_CALL","","SMBus 2.0",null,null],[17,"I2C_SMBUS_I2C_BLOCK_DATA","","",null,null],[17,"I2C_SMBUS_BLOCK_MAX","","As specified in SMBus standard",null,null],[17,"I2C_RETRIES","","number of times a device address should be polled when not acknowledging",null,null],[17,"I2C_TIMEOUT","","set timeout in units of 10 ms",null,null],[17,"I2C_SLAVE","","Use this slave address",null,null],[17,"I2C_SLAVE_FORCE","","Use this slave address, even if it is already in use by a driver!",null,null],[17,"I2C_TENBIT","","0 for 7 bit addrs, != 0 for 10 bit",null,null],[17,"I2C_FUNCS","","Get the adapter functionality mask",null,null],[17,"I2C_RDWR","","Combined R/W transfer (one STOP only)",null,null],[17,"I2C_PEC","","!= 0 to use PEC with SMBus",null,null],[17,"I2C_SMBUS","","SMBus transfer",null,null],[17,"I2C_RDWR_IOCTL_MAX_MSGS","","",null,null],[11,"eq","","",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"bool"}}],[11,"ne","","",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"bool"}}],[11,"clone","","",6,{"inputs":[{"name":"self"}],"output":{"name":"flags"}}],[11,"partial_cmp","","",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"generics":["ordering"],"name":"option"}}],[11,"lt","","",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"bool"}}],[11,"le","","",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"bool"}}],[11,"gt","","",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"bool"}}],[11,"ge","","",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"bool"}}],[11,"cmp","","",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"ordering"}}],[11,"hash","","",6,null],[11,"fmt","","",6,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"fmt","","",6,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"fmt","","",6,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"fmt","","",6,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"fmt","","",6,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[18,"RD","","read data, from slave to master",6,null],[18,"TEN","","this is a ten bit chip address",6,null],[18,"RECV_LEN","","length will be first received byte",6,null],[18,"NO_RD_ACK","","if I2C_FUNC_PROTOCOL_MANGLING",6,null],[18,"IGNORE_NACK","","if I2C_FUNC_PROTOCOL_MANGLING",6,null],[18,"REV_DIR_ADDR","","if I2C_FUNC_PROTOCOL_MANGLING",6,null],[18,"NOSTART","","I2C_FUNC_NOSTART",6,null],[18,"STOP","","if I2C_FUNC_PROTOCOL_MANGLING",6,null],[11,"empty","","Returns an empty set of flags.",6,{"inputs":[],"output":{"name":"flags"}}],[11,"all","","Returns the set containing all flags.",6,{"inputs":[],"output":{"name":"flags"}}],[11,"bits","","Returns the raw value of the flags currently stored.",6,{"inputs":[{"name":"self"}],"output":{"name":"u16"}}],[11,"from_bits","","Convert from underlying bit representation, unless that representation contains bits that do not correspond to a flag.",6,{"inputs":[{"name":"u16"}],"output":{"generics":["flags"],"name":"option"}}],[11,"from_bits_truncate","","Convert from underlying bit representation, dropping any bits that do not correspond to flags.",6,{"inputs":[{"name":"u16"}],"output":{"name":"flags"}}],[11,"is_empty","","Returns `true` if no flags are currently stored.",6,{"inputs":[{"name":"self"}],"output":{"name":"bool"}}],[11,"is_all","","Returns `true` if all flags are currently set.",6,{"inputs":[{"name":"self"}],"output":{"name":"bool"}}],[11,"intersects","","Returns `true` if there are flags common to both `self` and `other`.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"bool"}}],[11,"contains","","Returns `true` all of the flags in `other` are contained within `self`.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"bool"}}],[11,"insert","","Inserts the specified flags in-place.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":null}],[11,"remove","","Removes the specified flags in-place.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":null}],[11,"toggle","","Toggles the specified flags in-place.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":null}],[11,"set","","Inserts or removes the specified flags depending on the passed value.",6,{"inputs":[{"name":"self"},{"name":"flags"},{"name":"bool"}],"output":null}],[11,"bitor","","Returns the union of the two sets of flags.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"flags"}}],[11,"bitor_assign","","Adds the set of flags.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":null}],[11,"bitxor","","Returns the left flags, but with all the right flags toggled.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"flags"}}],[11,"bitxor_assign","","Toggles the set of flags.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":null}],[11,"bitand","","Returns the intersection between the two sets of flags.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"flags"}}],[11,"bitand_assign","","Disables all flags disabled in the set.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":null}],[11,"sub","","Returns the set difference of the two sets of flags.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":{"name":"flags"}}],[11,"sub_assign","","Disables all flags enabled in the set.",6,{"inputs":[{"name":"self"},{"name":"flags"}],"output":null}],[11,"not","","Returns the complement of this set of flags.",6,{"inputs":[{"name":"self"}],"output":{"name":"flags"}}],[11,"extend","","",6,{"inputs":[{"name":"self"},{"name":"t"}],"output":null}],[11,"from_iter","","",6,{"inputs":[{"name":"t"}],"output":{"name":"flags"}}],[11,"eq","","",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"bool"}}],[11,"ne","","",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"bool"}}],[11,"clone","","",7,{"inputs":[{"name":"self"}],"output":{"name":"functionality"}}],[11,"partial_cmp","","",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"generics":["ordering"],"name":"option"}}],[11,"lt","","",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"bool"}}],[11,"le","","",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"bool"}}],[11,"gt","","",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"bool"}}],[11,"ge","","",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"bool"}}],[11,"cmp","","",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"ordering"}}],[11,"hash","","",7,null],[11,"fmt","","",7,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"fmt","","",7,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"fmt","","",7,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"fmt","","",7,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"fmt","","",7,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[18,"I2C","","",7,null],[18,"TENBIT_ADDR","","",7,null],[18,"PROTOCOL_MANGLING","","I2C_M_IGNORE_NAK etc.",7,null],[18,"SMBUS_PEC","","",7,null],[18,"NOSTART","","I2C_M_NOSTART",7,null],[18,"SLAVE","","",7,null],[18,"SMBUS_BLOCK_PROC_CALL","","SMBus 2.0",7,null],[18,"SMBUS_QUICK","","",7,null],[18,"SMBUS_READ_BYTE","","",7,null],[18,"SMBUS_WRITE_BYTE","","",7,null],[18,"SMBUS_READ_BYTE_DATA","","",7,null],[18,"SMBUS_WRITE_BYTE_DATA","","",7,null],[18,"SMBUS_READ_WORD_DATA","","",7,null],[18,"SMBUS_WRITE_WORD_DATA","","",7,null],[18,"SMBUS_PROC_CALL","","",7,null],[18,"SMBUS_READ_BLOCK_DATA","","",7,null],[18,"SMBUS_WRITE_BLOCK_DATA","","",7,null],[18,"SMBUS_READ_I2C_BLOCK","","I2C-like block xfer",7,null],[18,"SMBUS_WRITE_I2C_BLOCK","","w/ 1-byte reg. addr.",7,null],[18,"SMBUS_HOST_NOTIFY","","",7,null],[18,"SMBUS_BYTE","","",7,null],[18,"SMBUS_BYTE_DATA","","",7,null],[18,"SMBUS_WORD_DATA","","",7,null],[18,"SMBUS_BLOCK_DATA","","",7,null],[18,"SMBUS_I2C_BLOCK","","",7,null],[18,"SMBUS_EMUL","","",7,null],[11,"empty","","Returns an empty set of flags.",7,{"inputs":[],"output":{"name":"functionality"}}],[11,"all","","Returns the set containing all flags.",7,{"inputs":[],"output":{"name":"functionality"}}],[11,"bits","","Returns the raw value of the flags currently stored.",7,{"inputs":[{"name":"self"}],"output":{"name":"u32"}}],[11,"from_bits","","Convert from underlying bit representation, unless that representation contains bits that do not correspond to a flag.",7,{"inputs":[{"name":"u32"}],"output":{"generics":["functionality"],"name":"option"}}],[11,"from_bits_truncate","","Convert from underlying bit representation, dropping any bits that do not correspond to flags.",7,{"inputs":[{"name":"u32"}],"output":{"name":"functionality"}}],[11,"is_empty","","Returns `true` if no flags are currently stored.",7,{"inputs":[{"name":"self"}],"output":{"name":"bool"}}],[11,"is_all","","Returns `true` if all flags are currently set.",7,{"inputs":[{"name":"self"}],"output":{"name":"bool"}}],[11,"intersects","","Returns `true` if there are flags common to both `self` and `other`.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"bool"}}],[11,"contains","","Returns `true` all of the flags in `other` are contained within `self`.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"bool"}}],[11,"insert","","Inserts the specified flags in-place.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":null}],[11,"remove","","Removes the specified flags in-place.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":null}],[11,"toggle","","Toggles the specified flags in-place.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":null}],[11,"set","","Inserts or removes the specified flags depending on the passed value.",7,{"inputs":[{"name":"self"},{"name":"functionality"},{"name":"bool"}],"output":null}],[11,"bitor","","Returns the union of the two sets of flags.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"functionality"}}],[11,"bitor_assign","","Adds the set of flags.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":null}],[11,"bitxor","","Returns the left flags, but with all the right flags toggled.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"functionality"}}],[11,"bitxor_assign","","Toggles the set of flags.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":null}],[11,"bitand","","Returns the intersection between the two sets of flags.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"functionality"}}],[11,"bitand_assign","","Disables all flags disabled in the set.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":null}],[11,"sub","","Returns the set difference of the two sets of flags.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":{"name":"functionality"}}],[11,"sub_assign","","Disables all flags enabled in the set.",7,{"inputs":[{"name":"self"},{"name":"functionality"}],"output":null}],[11,"not","","Returns the complement of this set of flags.",7,{"inputs":[{"name":"self"}],"output":{"name":"functionality"}}],[11,"extend","","",7,{"inputs":[{"name":"self"},{"name":"t"}],"output":null}],[11,"from_iter","","",7,{"inputs":[{"name":"t"}],"output":{"name":"functionality"}}],[11,"clone","","",4,{"inputs":[{"name":"self"}],"output":{"name":"smbusreadwrite"}}],[11,"fmt","","",4,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"partial_cmp","","",4,{"inputs":[{"name":"self"},{"name":"smbusreadwrite"}],"output":{"generics":["ordering"],"name":"option"}}],[11,"cmp","","",4,{"inputs":[{"name":"self"},{"name":"smbusreadwrite"}],"output":{"name":"ordering"}}],[11,"eq","","",4,{"inputs":[{"name":"self"},{"name":"smbusreadwrite"}],"output":{"name":"bool"}}],[11,"hash","","",4,null],[11,"clone","","",5,{"inputs":[{"name":"self"}],"output":{"name":"smbustransaction"}}],[11,"fmt","","",5,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"partial_cmp","","",5,{"inputs":[{"name":"self"},{"name":"smbustransaction"}],"output":{"generics":["ordering"],"name":"option"}}],[11,"cmp","","",5,{"inputs":[{"name":"self"},{"name":"smbustransaction"}],"output":{"name":"ordering"}}],[11,"eq","","",5,{"inputs":[{"name":"self"},{"name":"smbustransaction"}],"output":{"name":"bool"}}],[11,"hash","","",5,null],[11,"clone","","",0,{"inputs":[{"name":"self"}],"output":{"name":"i2c_msg"}}],[11,"fmt","","",0,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"clone","","",1,{"inputs":[{"name":"self"}],"output":{"name":"i2c_rdwr_ioctl_data"}}],[11,"fmt","","",1,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"clone","","",2,{"inputs":[{"name":"self"}],"output":{"name":"i2c_smbus_data"}}],[11,"from_byte","","",2,{"inputs":[{"name":"u8"}],"output":{"name":"self"}}],[11,"from_word","","",2,{"inputs":[{"name":"u16"}],"output":{"name":"self"}}],[11,"from_block","","",2,null],[11,"byte","","",2,{"inputs":[{"name":"self"}],"output":{"name":"u8"}}],[11,"set_byte","","",2,{"inputs":[{"name":"self"},{"name":"u8"}],"output":null}],[11,"word","","",2,{"inputs":[{"name":"self"}],"output":{"name":"u16"}}],[11,"set_word","","",2,{"inputs":[{"name":"self"},{"name":"u16"}],"output":null}],[11,"block","","",2,{"inputs":[{"name":"self"}],"output":{"name":"option"}}],[11,"block_mut","","",2,{"inputs":[{"name":"self"}],"output":{"name":"option"}}],[11,"set_block","","",2,null],[11,"fmt","","",2,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}],[11,"default","","",2,{"inputs":[],"output":{"name":"self"}}],[11,"clone","","",3,{"inputs":[{"name":"self"}],"output":{"name":"i2c_smbus_ioctl_data"}}],[11,"fmt","","",3,{"inputs":[{"name":"self"},{"name":"formatter"}],"output":{"name":"result"}}]],"paths":[[3,"i2c_msg"],[3,"i2c_rdwr_ioctl_data"],[3,"i2c_smbus_data"],[3,"i2c_smbus_ioctl_data"],[4,"SmbusReadWrite"],[4,"SmbusTransaction"],[3,"Flags"],[3,"Functionality"]]};
initSearch(searchIndex);