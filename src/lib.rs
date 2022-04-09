//#![deny(missing_docs)]
#![doc(html_root_url = "http://arcnmx.github.io/i2c-linux-sys-rs/")]

#[macro_use]
extern crate bitflags;
extern crate byteorder;
extern crate libc;

use std::{fmt, io, ptr, mem, cmp};
use std::os::unix::io::RawFd;
use byteorder::{NativeEndian, ByteOrder};
use libc::c_int;

bitflags! {
    pub struct Flags: u16 {
        /// read data, from slave to master
        const RD = I2C_M_RD;
        /// this is a ten bit chip address
        const TEN = I2C_M_TEN;
        /// length will be first received byte
        const RECV_LEN = I2C_M_RECV_LEN;
        /// if I2C_FUNC_PROTOCOL_MANGLING
        const NO_RD_ACK = I2C_M_NO_RD_ACK;
        /// if I2C_FUNC_PROTOCOL_MANGLING
        const IGNORE_NACK = I2C_M_IGNORE_NAK;
        /// if I2C_FUNC_PROTOCOL_MANGLING
        const REV_DIR_ADDR = I2C_M_REV_DIR_ADDR;
        /// I2C_FUNC_NOSTART
        const NO_START = I2C_M_NOSTART;
        /// if I2C_FUNC_PROTOCOL_MANGLING
        const STOP = I2C_M_STOP;
    }
}

/// read data, from slave to master
pub const I2C_M_RD: u16 = 0x0001;
/// this is a ten bit chip address
pub const I2C_M_TEN: u16 = 0x0010;
/// length will be first received byte
pub const I2C_M_RECV_LEN: u16 = 0x0400;
/// if `I2C_FUNC_PROTOCOL_MANGLING`
pub const I2C_M_NO_RD_ACK: u16 = 0x0800;
/// if `I2C_FUNC_PROTOCOL_MANGLING`
pub const I2C_M_IGNORE_NAK: u16 = 0x1000;
/// if `I2C_FUNC_PROTOCOL_MANGLING`
pub const I2C_M_REV_DIR_ADDR: u16 = 0x2000;
/// if `I2C_FUNC_NOSTART`
pub const I2C_M_NOSTART: u16 = 0x4000;
/// if `I2C_FUNC_PROTOCOL_MANGLING`
pub const I2C_M_STOP: u16 = 0x8000;

bitflags! {
    /// To determine what functionality is present
    pub struct Functionality: u32 {
        /// Plain i2c-level commands (`I2C_RDWR`)
        ///
        /// Pure SMBus adapters typically can not do these.
        const I2C = I2C_FUNC_I2C;
        /// Handles the 10-bit address extensions
        const TENBIT_ADDR = I2C_FUNC_10BIT_ADDR;
        /// I2C_M_IGNORE_NAK etc.
        const PROTOCOL_MANGLING = I2C_FUNC_PROTOCOL_MANGLING;
        const SMBUS_PEC = I2C_FUNC_SMBUS_PEC;
        /// I2C_M_NOSTART
        const NO_START = I2C_FUNC_NOSTART;
        const SLAVE = I2C_FUNC_SLAVE;
        /// SMBus 2.0
        const SMBUS_BLOCK_PROC_CALL = I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
        const SMBUS_QUICK = I2C_FUNC_SMBUS_QUICK;
        const SMBUS_READ_BYTE = I2C_FUNC_SMBUS_READ_BYTE;
        const SMBUS_WRITE_BYTE = I2C_FUNC_SMBUS_WRITE_BYTE;
        const SMBUS_READ_BYTE_DATA = I2C_FUNC_SMBUS_READ_BYTE_DATA;
        const SMBUS_WRITE_BYTE_DATA = I2C_FUNC_SMBUS_WRITE_BYTE_DATA;
        const SMBUS_READ_WORD_DATA = I2C_FUNC_SMBUS_READ_WORD_DATA;
        const SMBUS_WRITE_WORD_DATA = I2C_FUNC_SMBUS_WRITE_WORD_DATA;
        const SMBUS_PROC_CALL = I2C_FUNC_SMBUS_PROC_CALL;
        const SMBUS_READ_BLOCK_DATA = I2C_FUNC_SMBUS_READ_BLOCK_DATA;
        const SMBUS_WRITE_BLOCK_DATA = I2C_FUNC_SMBUS_WRITE_BLOCK_DATA;
        /// I2C-like block xfer
        const SMBUS_READ_I2C_BLOCK = I2C_FUNC_SMBUS_READ_I2C_BLOCK;
        /// w/ 1-byte reg. addr.
        const SMBUS_WRITE_I2C_BLOCK = I2C_FUNC_SMBUS_WRITE_I2C_BLOCK;
        const SMBUS_HOST_NOTIFY = I2C_FUNC_SMBUS_HOST_NOTIFY;

        const SMBUS_BYTE = I2C_FUNC_SMBUS_BYTE;
        const SMBUS_BYTE_DATA = I2C_FUNC_SMBUS_BYTE_DATA;
        const SMBUS_WORD_DATA = I2C_FUNC_SMBUS_WORD_DATA;
        const SMBUS_BLOCK_DATA = I2C_FUNC_SMBUS_BLOCK_DATA;
        const SMBUS_I2C_BLOCK = I2C_FUNC_SMBUS_I2C_BLOCK;

        const SMBUS_EMUL = I2C_FUNC_SMBUS_EMUL;
    }
}

pub const I2C_FUNC_I2C: u32 = 0x00000001;
pub const I2C_FUNC_10BIT_ADDR: u32 = 0x00000002;
/// `I2C_M_IGNORE_NAK` etc.
pub const I2C_FUNC_PROTOCOL_MANGLING: u32 = 0x00000004;
pub const I2C_FUNC_SMBUS_PEC: u32 = 0x00000008;
/// `I2C_M_NOSTART`
pub const I2C_FUNC_NOSTART: u32 = 0x00000010;
pub const I2C_FUNC_SLAVE: u32 = 0x00000020;

/// SMBus 2.0
pub const I2C_FUNC_SMBUS_BLOCK_PROC_CALL: u32 = 0x00008000;
pub const I2C_FUNC_SMBUS_QUICK: u32 = 0x00010000;
pub const I2C_FUNC_SMBUS_READ_BYTE: u32 = 0x00020000;
pub const I2C_FUNC_SMBUS_WRITE_BYTE: u32 = 0x00040000;
pub const I2C_FUNC_SMBUS_READ_BYTE_DATA: u32 = 0x00080000;
pub const I2C_FUNC_SMBUS_WRITE_BYTE_DATA: u32 = 0x00100000;
pub const I2C_FUNC_SMBUS_READ_WORD_DATA: u32 = 0x00200000;
pub const I2C_FUNC_SMBUS_WRITE_WORD_DATA: u32 = 0x00400000;
pub const I2C_FUNC_SMBUS_PROC_CALL: u32 = 0x00800000;
pub const I2C_FUNC_SMBUS_READ_BLOCK_DATA: u32 = 0x01000000;
pub const I2C_FUNC_SMBUS_WRITE_BLOCK_DATA: u32 = 0x02000000;
/// I2C-like block xfer
pub const I2C_FUNC_SMBUS_READ_I2C_BLOCK: u32 = 0x04000000;
/// w/ 1-byte reg. addr.
pub const I2C_FUNC_SMBUS_WRITE_I2C_BLOCK: u32 = 0x08000000;
pub const I2C_FUNC_SMBUS_HOST_NOTIFY: u32 = 0x10000000;

pub const I2C_FUNC_SMBUS_BYTE: u32 = I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE;
pub const I2C_FUNC_SMBUS_BYTE_DATA: u32 = I2C_FUNC_SMBUS_READ_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_BYTE_DATA;
pub const I2C_FUNC_SMBUS_WORD_DATA: u32 = I2C_FUNC_SMBUS_READ_WORD_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA;
pub const I2C_FUNC_SMBUS_BLOCK_DATA: u32 = I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_WRITE_BLOCK_DATA;
pub const I2C_FUNC_SMBUS_I2C_BLOCK: u32 = I2C_FUNC_SMBUS_READ_I2C_BLOCK | I2C_FUNC_SMBUS_WRITE_I2C_BLOCK;

pub const I2C_FUNC_SMBUS_EMUL: u32 = I2C_FUNC_SMBUS_QUICK |
    I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
    I2C_FUNC_SMBUS_PROC_CALL |
    I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | I2C_FUNC_SMBUS_I2C_BLOCK |
    I2C_FUNC_SMBUS_PEC;

/// `i2c_smbus_xfer` read or write markers
#[derive(Copy, Clone, Debug, PartialOrd, Ord, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum SmbusReadWrite {
    Read = 1,
    Write = 0,
}

pub const I2C_SMBUS_READ: SmbusReadWrite = SmbusReadWrite::Read;
pub const I2C_SMBUS_WRITE: SmbusReadWrite = SmbusReadWrite::Write;

/// SMBus transaction types (size parameter in the above functions)
///
/// Note: these no longer correspond to the (arbitrary) PIIX4 internal codes!
#[derive(Copy, Clone, Debug, PartialOrd, Ord, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum SmbusTransaction {
    Quick = I2C_SMBUS_QUICK,
    Byte = I2C_SMBUS_BYTE,
    ByteData = I2C_SMBUS_BYTE_DATA,
    WordData = I2C_SMBUS_WORD_DATA,
    ProcCall = I2C_SMBUS_PROC_CALL,
    BlockData = I2C_SMBUS_BLOCK_DATA,
    I2cBlockBroken = I2C_SMBUS_I2C_BLOCK_BROKEN,
    /// SMBus 2.0
    BlockProcCall = I2C_SMBUS_BLOCK_PROC_CALL,
    I2cBlockData = I2C_SMBUS_I2C_BLOCK_DATA,
}

pub const I2C_SMBUS_QUICK: u32 = 0;
pub const I2C_SMBUS_BYTE: u32 = 1;
pub const I2C_SMBUS_BYTE_DATA: u32 = 2;
pub const I2C_SMBUS_WORD_DATA: u32 = 3;
pub const I2C_SMBUS_PROC_CALL: u32 = 4;
pub const I2C_SMBUS_BLOCK_DATA: u32 = 5;
pub const I2C_SMBUS_I2C_BLOCK_BROKEN: u32 = 6;
/// SMBus 2.0
pub const I2C_SMBUS_BLOCK_PROC_CALL: u32 = 7;
pub const I2C_SMBUS_I2C_BLOCK_DATA: u32 = 8;

/// As specified in SMBus standard
pub const I2C_SMBUS_BLOCK_MAX: usize = 32;

#[repr(C)]
#[derive(Copy, Clone, Debug)]
/// an I2C transaction segment beginning with START
///
/// An i2c_msg is the low level representation of one segment of an I2C
/// transaction. It is visible to drivers in the `i2c_transfer()` procedure,
/// to userspace from i2c-dev, and to I2C adapter drivers through the
/// `i2c_adapter.master_xfer()` method.
///
/// Except when I2C "protocol mangling" is used, all I2C adapters implement
/// the standard rules for I2C transactions. Each transaction begins with a
/// START. That is followed by the slave address, and a bit encoding read
/// versus write. Then follow all the data bytes, possibly including a byte
/// with SMBus PEC. The transfer terminates with a NAK, or when all those
/// bytes have been transferred and ACKed. If this is the last message in a
/// group, it is followed by a STOP. Otherwise it is followed by the next
/// `i2c_msg` transaction segment, beginning with a (repeated) START.
///
/// Alternatively, when the adapter supports `I2C_FUNC_PROTOCOL_MANGLING` then
/// passing certain `flags` may have changed those standard protocol behaviors.
/// Those flags are only for use with broken/nonconforming slaves, and with
/// adapters which are known to support the specific mangling options they
/// need (one or more of `IGNORE_NACK`, `NO_RD_ACK`, `NOSTART`, and `REV_DIR_ADDR`).
pub struct i2c_msg {
    /// Slave address, either seven or ten bits.
    ///
    /// When this is a ten
    /// bit address, `I2C_M_TEN` must be set in `flags` and the adapter
    /// must support `I2C_FUNC_10BIT_ADDR`.
    pub addr: u16,
    /// `I2C_M_RD` is handled by all adapters.
    ///
    /// No other flags may be
    /// provided unless the adapter exported the relevant `I2C_FUNC_*`
    /// flags through `i2c_get_functionality()`.
    pub flags: Flags,
    /// Number of data bytes in `buf` being read from or written to the
    /// I2C slave address.
    ///
    /// For read transactions where `I2C_M_RECV_LEN`
    /// is set, the caller guarantees that this buffer can hold up to
    /// 32 bytes in addition to the initial length byte sent by the
    /// slave (plus, if used, the SMBus PEC); and this value will be
    /// incremented by the number of block data bytes received.
    pub len: u16,
    /// The buffer into which data is read, or from which it's written.
    pub buf: *mut u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
/// This is the structure as used in the `I2C_RDWR` ioctl call
pub struct i2c_rdwr_ioctl_data {
    /// ptr to array of simple messages
    pub msgs: *mut i2c_msg,
    /// number of messages to exchange
    pub nmsgs: c_int,
}

#[repr(C)]
#[derive(Copy, Clone)]
/// Data for SMBus Messages
pub struct i2c_smbus_data {
    _alignment: [u16; 0],
    /// block[0] is used for length and one more for user-space compatibility
    pub block: [u8; I2C_SMBUS_BLOCK_MAX + 2],
}

impl i2c_smbus_data {
    pub fn from_byte(v: u8) -> Self {
        let mut data = Self::default();
        data.set_byte(v);
        data
    }

    pub fn from_word(v: u16) -> Self {
        let mut data = Self::default();
        data.set_word(v);
        data
    }

    pub fn from_block(v: &[u8]) -> Self {
        let mut data = Self::default();
        data.set_block(v);
        data
    }

    pub fn byte(&self) -> u8 {
        self.block[0]
    }

    pub fn set_byte(&mut self, v: u8) {
        self.block[0] = v;
    }

    pub fn word(&self) -> u16 {
        NativeEndian::read_u16(&self.block[..2])
    }

    pub fn set_word(&mut self, v: u16) {
        NativeEndian::write_u16(&mut self.block[..2], v)
    }

    pub fn block(&self) -> Option<&[u8]> {
        let len = self.block[0] as usize;
        if len <= I2C_SMBUS_BLOCK_MAX {
            Some(&self.block[1..1 + len])
        } else {
            None
        }
    }

    pub fn block_mut(&mut self) -> Option<&mut [u8]> {
        let len = self.block[0] as usize;
        if len <= I2C_SMBUS_BLOCK_MAX {
            Some(&mut self.block[1..1 + len])
        } else {
            None
        }
    }

    pub fn set_block(&mut self, v: &[u8]) {
        assert!(v.len() <= I2C_SMBUS_BLOCK_MAX);

        self.block[0] = v.len() as u8;
        self.block[1..1 + v.len()].copy_from_slice(v);
    }
}

impl fmt::Debug for i2c_smbus_data {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("i2c_smbus_data")
            .field("byte", &self.byte())
            .field("word", &self.word())
            .field("block", &self.block())
            .finish()
    }
}

impl Default for i2c_smbus_data {
    fn default() -> Self {
        unsafe { mem::zeroed() }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
/// This is the structure as used in the I2C_SMBUS ioctl call
pub struct i2c_smbus_ioctl_data {
    pub read_write: SmbusReadWrite,
    pub command: u8,
    pub size: SmbusTransaction,
    pub data: *mut i2c_smbus_data,
}

/// number of times a device address should be polled when not acknowledging
pub const I2C_RETRIES: u16 = 0x0701;
/// set timeout in units of 10 ms
pub const I2C_TIMEOUT: u16 = 0x0702;

/// Use this slave address
///
/// NOTE: Slave address is 7 or 10 bits, but 10-bit addresses
/// are NOT supported! (due to code brokenness)
pub const I2C_SLAVE: u16 = 0x0703;
/// Use this slave address, even if it is already in use by a driver!
pub const I2C_SLAVE_FORCE: u16 = 0x0706;
/// 0 for 7 bit addrs, != 0 for 10 bit
pub const I2C_TENBIT: u16 = 0x0704;

/// Get the adapter functionality mask
pub const I2C_FUNCS: u16 = 0x0705;

/// Combined R/W transfer (one STOP only)
pub const I2C_RDWR: u16 = 0x0707;

/// != 0 to use PEC with SMBus
pub const I2C_PEC: u16 = 0x0708;
/// SMBus transfer
pub const I2C_SMBUS: u16 = 0x0720;

pub const I2C_RDWR_IOCTL_MAX_MSGS: usize = 42;

pub mod ioctls {
    use libc::{ioctl, c_int, c_ulong};
    use std::os::unix::io::RawFd;
    use std::io;

    #[inline]
    fn ioctl_result(res: c_int) -> io::Result<c_int> {
        if res == -1 {
            Err(io::Error::last_os_error())
        } else {
            Ok(res)
        }
    }

    pub unsafe fn i2c_retries(fd: RawFd, value: c_int) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_RETRIES as _, value))
    }

    pub unsafe fn i2c_timeout(fd: RawFd, value: c_int) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_TIMEOUT as _, value))
    }

    pub unsafe fn i2c_slave(fd: RawFd, value: c_int) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_SLAVE as _, value))
    }

    pub unsafe fn i2c_slave_force(fd: RawFd, value: c_int) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_SLAVE_FORCE as _, value))
    }

    pub unsafe fn i2c_tenbit(fd: RawFd, value: c_int) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_TENBIT as _, value))
    }

    pub unsafe fn i2c_funcs(fd: RawFd, value: *mut c_ulong) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_FUNCS as _, value))
    }

    pub unsafe fn i2c_rdwr(fd: RawFd, value: *mut super::i2c_rdwr_ioctl_data) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_RDWR as _, value))
    }

    pub unsafe fn i2c_pec(fd: RawFd, value: c_int) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_PEC as _, value))
    }

    pub unsafe fn i2c_smbus(fd: RawFd, value: *mut super::i2c_smbus_ioctl_data) -> io::Result<c_int> {
        ioctl_result(ioctl(fd, super::I2C_SMBUS as _, value))
    }
}

/// `I2C_RETRIES`
#[inline]
pub fn i2c_set_retries(fd: RawFd, value: usize) -> io::Result<()> {
    unsafe {
        ioctls::i2c_retries(fd, value as _).map(drop)
    }
}

/// `I2C_TIMEOUT`
#[inline]
pub fn i2c_set_timeout_ms(fd: RawFd, value: usize) -> io::Result<()> {
    unsafe {
        ioctls::i2c_timeout(fd, (value / 10) as _).map(drop)
    }
}

/// `I2C_SLAVE` and `I2C_SLAVE_FORCE`
#[inline]
pub fn i2c_set_slave_address(fd: RawFd, address: u16, force: bool) -> io::Result<()> {
    unsafe {
        if force {
            ioctls::i2c_slave(fd, address as _)
        } else {
            ioctls::i2c_slave_force(fd, address as _)
        }.map(drop)
    }
}

/// `I2C_TENBIT`
#[inline]
pub fn i2c_set_slave_address_10bit(fd: RawFd, tenbit: bool) -> io::Result<()> {
    unsafe {
        ioctls::i2c_tenbit(fd, if tenbit { 1 } else { 0 }).map(drop)
    }
}

/// `I2C_FUNCS`
#[inline]
pub fn i2c_get_functionality(fd: RawFd) -> io::Result<Functionality> {
    unsafe {
        let mut res = 0;
        ioctls::i2c_funcs(fd, &mut res)
            .map(|_| Functionality::from_bits_truncate(res as _))
    }
}

/// `I2C_PEC`
#[inline]
pub fn i2c_pec(fd: RawFd, pec: bool) -> io::Result<()> {
    unsafe {
        ioctls::i2c_pec(fd, if pec { 1 } else { 0 }).map(drop)
    }
}

/// `I2C_RDWR`
#[inline]
pub unsafe fn i2c_rdwr(fd: RawFd, msgs: &mut [i2c_msg]) -> io::Result<()> {
    let mut data = i2c_rdwr_ioctl_data {
        msgs: msgs.as_mut_ptr(),
        nmsgs: msgs.len() as _,
    };
    ioctls::i2c_rdwr(fd, &mut data).map(drop)
}

/// `I2C_SMBUS`
#[inline]
pub unsafe fn i2c_smbus(fd: RawFd, data: &mut i2c_smbus_ioctl_data) -> io::Result<()> {
    ioctls::i2c_smbus(fd, data).map(drop)
}

pub fn i2c_smbus_write_quick(fd: RawFd, value: SmbusReadWrite) -> io::Result<()> {
    unsafe {
        i2c_smbus(fd, &mut i2c_smbus_ioctl_data {
            read_write: value,
            command: 0,
            size: SmbusTransaction::Quick,
            data: ptr::null_mut(),
        })
    }
}

pub fn i2c_smbus_read_byte(fd: RawFd) -> io::Result<u8> {
    unsafe {
        let mut data = i2c_smbus_data::default();
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Read,
            command: 0,
            size: SmbusTransaction::Byte,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
            .map(|_| data.byte())
    }
}

pub fn i2c_smbus_write_byte(fd: RawFd, value: u8) -> io::Result<()> {
    unsafe {
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Write,
            command: value,
            size: SmbusTransaction::Byte,
            data: ptr::null_mut(),
        };
        i2c_smbus(fd, &mut ioctl)
    }
}

pub fn i2c_smbus_read_byte_data(fd: RawFd, command: u8) -> io::Result<u8> {
    unsafe {
        let mut data = i2c_smbus_data::default();
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Read,
            command: command,
            size: SmbusTransaction::ByteData,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
            .map(|_| data.byte())
    }
}

pub fn i2c_smbus_write_byte_data(fd: RawFd, command: u8, value: u8) -> io::Result<()> {
    unsafe {
        let mut data = i2c_smbus_data::from_byte(value);
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Write,
            command: command,
            size: SmbusTransaction::ByteData,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
    }
}

pub fn i2c_smbus_read_word_data(fd: RawFd, command: u8) -> io::Result<u16> {
    unsafe {
        let mut data = i2c_smbus_data::default();
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Read,
            command: command,
            size: SmbusTransaction::WordData,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
            .map(|_| data.word())
    }
}

pub fn i2c_smbus_write_word_data(fd: RawFd, command: u8, value: u16) -> io::Result<()> {
    unsafe {
        let mut data = i2c_smbus_data::from_word(value);
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Write,
            command: command,
            size: SmbusTransaction::WordData,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
    }
}

pub fn i2c_smbus_process_call(fd: RawFd, command: u8, value: u16) -> io::Result<u16> {
    unsafe {
        let mut data = i2c_smbus_data::from_word(value);
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Write,
            command: command,
            size: SmbusTransaction::ProcCall,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
            .map(|_| data.word())
    }
}

pub fn i2c_smbus_read_block_data(fd: RawFd, command: u8, value: &mut [u8]) -> io::Result<usize> {
    unsafe {
        let mut data = i2c_smbus_data::default();
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Read,
            command: command,
            size: SmbusTransaction::BlockData,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
            .map(|_| {
                let block = data.block().expect("kernel provided an invalid block length");
                let len = cmp::min(block.len(), value.len());
                value[..len].copy_from_slice(&block[..len]);
                block.len()
            })
    }
}

pub fn i2c_smbus_write_block_data(fd: RawFd, command: u8, value: &[u8]) -> io::Result<()> {
    unsafe {
        let mut data = i2c_smbus_data::from_block(value);
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Write,
            command: command,
            size: SmbusTransaction::BlockData,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
    }
}

pub fn i2c_smbus_read_i2c_block_data(fd: RawFd, command: u8, value: &mut [u8]) -> io::Result<usize> {
    assert!(value.len() <= I2C_SMBUS_BLOCK_MAX);

    unsafe {
        let mut data = i2c_smbus_data::from_byte(value.len() as u8);
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Read,
            command: command,
            size: if value.len() == I2C_SMBUS_BLOCK_MAX { SmbusTransaction::I2cBlockBroken } else { SmbusTransaction::I2cBlockData },
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
            .map(|_| {
                let block = data.block().expect("kernel provided an invalid block length");
                let len = cmp::min(block.len(), value.len());
                value[..len].copy_from_slice(&block[..len]);
                block.len()
            })
    }
}

pub fn i2c_smbus_write_i2c_block_data(fd: RawFd, command: u8, value: &[u8]) -> io::Result<()> {
    unsafe {
        let mut data = i2c_smbus_data::from_block(value);
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Write,
            command: command,
            size: SmbusTransaction::I2cBlockBroken,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
    }
}

pub fn i2c_smbus_block_process_call(fd: RawFd, command: u8, write: &[u8], read: &mut [u8]) -> io::Result<usize> {
    assert!(read.len() <= I2C_SMBUS_BLOCK_MAX - 1);

    unsafe {
        let mut data = i2c_smbus_data::from_block(write);
        let mut ioctl = i2c_smbus_ioctl_data {
            read_write: SmbusReadWrite::Write,
            command: command,
            size: SmbusTransaction::BlockProcCall,
            data: &mut data,
        };
        i2c_smbus(fd, &mut ioctl)
            .map(|_| {
                let block = data.block().expect("kernel provided an invalid block length");
                let len = cmp::min(block.len(), read.len());
                read[..len].copy_from_slice(&block[..len]);
                block.len()
            })
    }
}
