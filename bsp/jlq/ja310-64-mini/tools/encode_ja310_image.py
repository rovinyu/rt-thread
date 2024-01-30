#!/usr/bin/python2
# SPDX-License-Identifier: GPL-2.0+
#-*- coding: UTF-8 -*-
"""
        @author qipu<qipu@leadcoretech.com> huangqingwei<huangqingwei@leadcoretech.com>
        @function:
           Encode images via signing and encryption.
"""
import base64
import ssl
import struct
import sys

from Crypto.PublicKey import RSA
from Crypto.Signature import PKCS1_v1_5
from Crypto.Hash import SHA256
from Crypto.Cipher import AES
from binascii import b2a_hex, a2b_hex

class LSEECrypt():
	def __init__(self, key):
		self.key = key
		self.mode = AES.MODE_CBC

	def encrypt(self, data):
		encryptor = AES.new(self.key, self.mode,
			b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
		block_size = 16
		remainder = len(data) % block_size
		if remainder != 0:
			data = data + ('\0' * (block_size - remainder))
		self.crypt_data = encryptor.encrypt(data)
		return self.crypt_data

	def decrypt(self, data):
		decryptor = AES.new(self.key, self.mode,
			b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
		plain_data = decryptor.decrypt(data)
		return plain_data

def extended_gcd(aa, bb):
	lastremainder, remainder = abs(aa), abs(bb)
	x, lastx, y, lasty = 0, 1, 1, 0
	while remainder:
		lastremainder, (quotient, remainder) = remainder, divmod(lastremainder, remainder)
		x, lastx = lastx - quotient*x, x
		y, lasty = lasty - quotient*y, y
	return lastremainder, lastx * (-1 if aa < 0 else 1), lasty * (-1 if bb < 0 else 1)

def modinv(a, m):
	g, x, y = extended_gcd(a, m)
	if g != 1:
		raise ValueError
	return x % m

def dump_key(key, f=None):
	#N
	N = key.n
	#nwords
	nwords = N.bit_length() / 32
	B = 0x100000000
	N0inv = B - modinv(N, B)

	R = 2 ** (N.bit_length())
	RR = (R * R) % N

	print 'dump public key:', nwords, '|', N0inv, '|',
	if f:
		f.write(struct.pack('<I',nwords))
		f.write(struct.pack('<I',N0inv))

	#N
	for i in xrange(nwords):
		n = N % B
		print hex(n),
		if f:
			f.write(struct.pack('<I',n))
		N = N/B
	#RR
	print '|',
	for i in xrange(nwords):
		rr = RR % B
		print rr,
		if f:
			f.write(struct.pack('<I',rr))
		RR = RR / B
	#E
	E = key.e
	print '|', E
	if f:
		f.write(struct.pack('<I',E))

# write n p to f, p should: 0 <= p < 2^32
def padding(f, n, p):
	for i in xrange(n):
		f.write(struct.pack('<I', p))

def get_image_id(image_type):
	if image_type == 'BL2':
		image_id = 'BL*2'
	elif image_type == 'Secure_Monitor':
		image_id = 'BL31'
	elif image_type == 'TEE_OS':
		image_id = 'BL32'
	elif image_type == 'Bootloader':
		image_id = 'BL33'
	elif image_type == 'Linux_Kernel':
		image_id = 'LNXK'
	elif image_type == 'ROOTFS':
		image_id = 'RTFS'
	elif image_type == 'TL':
		image_id = 'TLDR'
	return image_id

def crypt_image(image, crypt_key):
	if crypt_key == 'null':
		return image
	else:
		key = open(crypt_key, 'r').read(16)
		lseecrypt = LSEECrypt(key)
		crypted_data = lseecrypt.encrypt(image)
		return crypted_data

def set_encrypt_flag(crypt_key):
	if crypt_key == 'null':
		attr_encrypt_flag = 0b00
	else:
		attr_encrypt_flag = 0b01
	return attr_encrypt_flag

def set_image_attr(crypt_key):
	attr_encrypt_flag = set_encrypt_flag(crypt_key)
	return attr_encrypt_flag

def sign(src, key):
	digest = SHA256.new(src)
	print 'SHA256 of image data: ', digest.hexdigest()
	signer = PKCS1_v1_5.new(key)
	signature = signer.sign(digest)
	return signature

if __name__ == '__main__':
	if len(sys.argv) != 8:
		print 'Usage: ' + sys.argv[0] + ' BL2/Secure_Monitor/TEE_OS/Bootloader/Linux_Kernel/ROOTFS/TL <source_image> <dest_addr> \
<pubkey_file>/null <privkey_file>/null <aeskey_file>/null <image_version>'
		exit(1)

	src_file_type = sys.argv[1]
	src_file = sys.argv[2]
	dest_addr = long(sys.argv[3], 0)

	sign_pub_key = sys.argv[4]
	sign_priv_key = sys.argv[5]

	crypt_key = sys.argv[6]
	image_version = sys.argv[7]


	f_src = open(src_file, 'rb')
	f_out = open(src_file + '.enc', 'wb')

	image_crypted = crypt_image(f_src.read(), crypt_key)

	# Header define:
	#
	# struct image_header {
	# uint32 header_magic;
	# 	uint32 header_len;
	# 	uint32 header_version;
	# 	uint32 image_id;
	# 	uint32 image_attr;
	# 	uint32 image_counter;
	#	uint64 dest_addr;
	# 	uint32 image_offset;
	# 	uint32 image_len;
	# 	uint32 image_plain_len;
	# 	uint32 reserved[5];
	# }

	header_magic = 'IM*H'
	header_len = 64
	header_version = 1
	image_id = get_image_id(src_file_type)
	image_attr = set_image_attr(crypt_key)
	image_counter = int(image_version)

	image_offset = 4096
	image_len = len(image_crypted)
	image_plain_len = f_src.tell()

	print 'image_len: ', image_len
	print 'image_plain_len: ', image_plain_len

	header = struct.pack('<4sII4sIIQIII5I',header_magic, header_len, header_version,
			image_id, image_attr, image_counter, dest_addr,
			image_offset, image_len, image_plain_len,
			0, 0, 0, 0, 0)

	f_src.seek(0, 0)

	f_out.write(header)

	if sign_pub_key == 'null':
		# ROTPK is 524 bytes
		padding(f_out, 131, 0x0)

		header_image = header + image_crypted

		image_digest = SHA256.new(header_image)
		print 'SHA256 of image data: ', image_digest.hexdigest()
		f_out.write(image_digest.digest())
		# (4096 - 64 - 524 - 32) / 4
		padding(f_out, 869, 0x0)
	else:
		priv_key = RSA.importKey(open(sign_priv_key, "rb").read())
		rotpk = RSA.importKey(open(sign_pub_key, "rb").read())

		dump_key(rotpk, f_out)

		header_image = header + image_crypted

		signature = sign(header_image, priv_key)
		f_out.write(signature)
		# (4096 - 64 - 524 - 256) / 4
		padding(f_out, 813, 0x0)

	#write file
	f_out.write(image_crypted)
	f_out.close()
	f_src.close()

	print 'image_attr:', bin(image_attr), '\n'
