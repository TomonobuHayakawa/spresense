#ifndef _VTUN_COMPAT_SPRESENSE_H
#define _VTUN_COMPAT_SPRESENSE_H

#include <mbedtls/blowfish.h>
#include <mbedtls/cipher.h>

// openssl
#define BF_KEY mbedtls_blowfish_context
#define BF_ENCRYPT 1
#define BF_DECRYPT 0
#define EVP_CIPHER_CTX mbedtls_cipher_context_t
#define EVP_CIPHER mbedtls_cipher_type_t
#define ENGINE void

unsigned char *MD5(char *msg, size_t msg_len, unsigned char* out);
void RAND_bytes(char *buf, size_t buf_len);
void BF_set_key(BF_KEY *key, int len, const unsigned char *data);
void BF_ecb_encrypt(const unsigned char *in, unsigned char *out,
        BF_KEY *key, int enc);
void EVP_CIPHER_CTX_init(EVP_CIPHER_CTX *a);
int EVP_CIPHER_CTX_cleanup(EVP_CIPHER_CTX *a);
int EVP_EncryptInit_ex(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *type,
        ENGINE *impl, unsigned char *key, unsigned char *iv);
int EVP_DecryptInit_ex(EVP_CIPHER_CTX *ctx, const EVP_CIPHER *type,
        ENGINE *impl, unsigned char *key, unsigned char *iv);
int EVP_CIPHER_CTX_set_padding(EVP_CIPHER_CTX *x, int padding);
int EVP_EncryptUpdate(EVP_CIPHER_CTX *ctx, unsigned char *out,
         int *outl, const unsigned char *in, int inl);
int EVP_DecryptUpdate(EVP_CIPHER_CTX *ctx, unsigned char *out,
         int *outl, const unsigned char *in, int inl);
int EVP_CIPHER_CTX_set_key_length(EVP_CIPHER_CTX *x, int keylen);
const EVP_CIPHER *EVP_aes_256_ecb(void);
const EVP_CIPHER *EVP_aes_128_ecb(void);
const EVP_CIPHER *EVP_bf_ecb(void);
const EVP_CIPHER *EVP_aes_256_ofb(void);
const EVP_CIPHER *EVP_aes_256_cfb(void);
const EVP_CIPHER *EVP_aes_256_cbc(void);
const EVP_CIPHER *EVP_aes_128_ofb(void);
const EVP_CIPHER *EVP_aes_128_cfb(void);
const EVP_CIPHER *EVP_aes_128_cbc(void);
const EVP_CIPHER *EVP_bf_ofb(void);
const EVP_CIPHER *EVP_bf_cfb(void);
const EVP_CIPHER *EVP_bf_cbc(void);

// syslog
#define	LOG_PID		0x01	/* log the pid with each message */
//#define	LOG_CONS	0x02	/* log on the console if errors in sending */
//#define	LOG_ODELAY	0x04	/* delay open until first syslog() (default) */
#define	LOG_NDELAY	0x08	/* don't delay open */
//#define	LOG_NOWAIT	0x10	/* don't wait for console forks: DEPRECATED */
#define	LOG_PERROR	0x20	/* log to stderr as well */

void openlog(const char *ident, int option, int facility);
void closelog(void);

// signal
#define SIGHUP 1

#ifndef CONFIG_PSEUDOFS_SOFTLINKS
int link(FAR const char *path1, FAR const char *path2);
#endif

int getpriority(int which, id_t who);
int setpriority(int which, id_t who, int prio);

pid_t setsid(void);

#endif /* _VTUN_COMPAT_SPRESENSE_H */
