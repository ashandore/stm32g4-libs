//Wrappers required for ARM EABI support under clang
//GCC will generate these automatically, but clang doesn't.

#include <string.h>

void __aeabi_memcpy(void *dest, const void *src, size_t n);
void __aeabi_memmove(void *dest, const void *src, size_t n);
void __aeabi_memset(void *s, size_t n, int c);

// void __aeabi_memcpy(void *dest, const void *src, size_t n) { 
// 	(void)memcpy(dest, src, n); 
// } 

void __aeabi_memmove(void *dest, const void *src, size_t n) { 
	(void)memmove(dest, src, n); 
} 

void __aeabi_memset(void *s, size_t n, int c) { 
	(void)memset(s, c, n); 
} 

