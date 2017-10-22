#include "config.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <pthread.h>

#include <string.h>
#include <stdint.h>

//////////// BELOW is clement code, use c to support x11 instead of ASM /////////////////
#include "sph_groestl.h"
#include "sph_jh.h"
#include "sph_luffa.h"
#include "sph_cubehash.h"
#include "sph_echo.h"
/////////////////////////////////////////////////////////////////////////////////

#include "sph_shavite.h"
//-------
//#include "x5/sph_simd.h"
//-----simd vect128---------
//#include "nist.h"
//-----------
#include "sph_simd.h"

//----
#include "x6/blake.c"
#include "x6/bmw.c"
#include "x6/keccak.c"
#include "x6/skein.c"

static inline uint32_t be32dec(const void *pp)
{
	const uint8_t *p = (uint8_t const *)pp;
	return ((uint32_t)(p[3]) + ((uint32_t)(p[2]) << 8) +
	    ((uint32_t)(p[1]) << 16) + ((uint32_t)(p[0]) << 24));
}

static inline uint32_t le32dec(const void *pp)
{
	const uint8_t *p = (uint8_t const *)pp;
	return ((uint32_t)(p[0]) + ((uint32_t)(p[1]) << 8) +
	    ((uint32_t)(p[2]) << 16) + ((uint32_t)(p[3]) << 24));
}

static inline void be32enc(void *pp, uint32_t x)
{
	uint8_t *p = (uint8_t *)pp;
	p[3] = x & 0xff;
	p[2] = (x >> 8) & 0xff;
	p[1] = (x >> 16) & 0xff;
	p[0] = (x >> 24) & 0xff;
}

static inline void le32enc(void *pp, uint32_t x)
{
	uint8_t *p = (uint8_t *)pp;
	p[0] = x & 0xff;
	p[1] = (x >> 8) & 0xff;
	p[2] = (x >> 16) & 0xff;
	p[3] = (x >> 24) & 0xff;
}

/*define data alignment for different C compilers*/
#if defined(__GNUC__)
      #define DATA_ALIGN16(x) x __attribute__ ((aligned(16)))
#else
      #define DATA_ALIGN16(x) __declspec(align(16)) x
#endif

//#define DUMP_INFO_PERSTEP

static void grsHash(unsigned char *input, unsigned char *outhash)
{
	sph_groestl512_context cc;
	sph_groestl512_init(&cc);
	sph_groestl512(&cc,input,64);
	sph_groestl512_close(&cc,outhash);
}

static void jhHash(unsigned char *input, unsigned char *outhash)
{
	sph_jh_context cc;
	sph_jh512_init((void *)&cc);
	sph_jh512(&cc,input,64);
	sph_jh512_close(&cc,outhash);
}

static void luffaHash(unsigned char *input, unsigned char *outhash)
{
	sph_luffa512_context cc;
	sph_luffa512_init(&cc);
	sph_luffa512(&cc,input,64);
	sph_luffa512_close(&cc,outhash);
}

static void echoHash(unsigned char *input, unsigned char *outhash)
{
	sph_echo_big_context cc;
	sph_echo512_init(&cc);
	sph_echo512(&cc,input,64);
	sph_echo512_close(&cc,outhash);
}

static void cubeHash(unsigned char *input, unsigned char *outhash)
{
	sph_cubehash_context cc;
	sph_cubehash512_init(&cc);
	sph_cubehash512(&cc,input,64);
	sph_cubehash512_close(&cc,outhash);
}

static void simdHash(unsigned char *input, unsigned char *outhash)
{
	sph_simd_big_context cc;
	sph_simd512_init(&cc);
	sph_simd512(&cc,input,64);
	sph_simd512_close(&cc,outhash);
}

void Xhash(void *state, const void *input)
{
	//printf("-----------------Xhash-------------------\n");

    DATA_ALIGN16(unsigned char hashbuf[128]);
    DATA_ALIGN16(size_t hashptr);
    DATA_ALIGN16(sph_u64 hashctA);
    DATA_ALIGN16(sph_u64 hashctB);

    int speedrun[] = {0, 1, 3, 4, 6, 7 };
    int i;
    DATA_ALIGN16(unsigned char hash[128]);
    /* proably not needed */
    memset(hash, 0, 128);
	
// blake1-bmw2-grs3-skein4-jh5-keccak6-luffa7-cubehash8-shavite9-simd10-echo11

#ifdef DUMP_INFO_PERSTEP
	char strInfo[1024];
	unsigned char *pByte;
	uint64_t tmpValue;
	char ret_line[3]={'\r','\n','\0'};
	
	printf("-----------------X11 input-------------------\n");
	pByte=(unsigned char *)input;
	BinToHexStr(strInfo,pByte,128);
	printf("input=%s\n",strInfo);
#endif

	//---blake1---
    DECL_BLK;
    BLK_I;
    BLK_W;
    BLK_C;

#ifdef DUMP_INFO_PERSTEP
	printf("-----------------blake output-------------------\n");
	pByte=(unsigned char *)hash;
	BinToHexStr(strInfo,pByte,128);	// defined unsigned char hash[128]
	printf("hash=%s\n",strInfo);
#endif

//---bmw2---
	DECL_BMW;
	BMW_I;
	BMW_U;
	#define M(x)    sph_dec64le_aligned(data + 8 * (x))
	#define H(x)    (h[x])
	#define dH(x)   (dh[x])
    BMW_C;
	#undef M
	#undef H
	#undef dH
	
#ifdef DUMP_INFO_PERSTEP
	printf("-----------------bmw output-------------------\n");
	pByte=(unsigned char *)hash;
	BinToHexStr(strInfo,pByte,128); // defined unsigned char hash[128]
	printf("hash=%s\n",strInfo);
#endif


//---grs3 ---
	grsHash(hash,hash);

#ifdef DUMP_INFO_PERSTEP
	printf("-----------------grs output-------------------\n");
	pByte=(unsigned char *)hash;
	BinToHexStr(strInfo,pByte,128);	// defined unsigned char hash[128]
	printf("hash=%s\n",strInfo);
#endif

//---skein4---          
	DECL_SKN;
	SKN_I;
	SKN_U;
	SKN_C; 
	
#ifdef DUMP_INFO_PERSTEP
	printf("-----------------skein output-------------------\n");
	pByte=(unsigned char *)hash;
	BinToHexStr(strInfo,pByte,128); // defined unsigned char hash[128]
	printf("hash=%s\n",strInfo);
#endif

//---jh5---    
	jhHash(hash,hash);

#ifdef DUMP_INFO_PERSTEP
	printf("-----------------jh output-------------------\n");
	pByte=(unsigned char *)hash;
	BinToHexStr(strInfo,pByte,128); // defined unsigned char hash[128]
	printf("hash=%s\n",strInfo);
#endif

//---keccak6---       
	DECL_KEC;
	KEC_I;
	KEC_U;
	KEC_C;

#ifdef DUMP_INFO_PERSTEP
	 printf("-----------------keccak output-------------------\n");
	 pByte=(unsigned char *)hash;
	 BinToHexStr(strInfo,pByte,128); // defined unsigned char hash[128]
	 printf("hash=%s\n",strInfo);
#endif


//---local simd var ---
	//uint32_t hashA[16], hashB[16];	

	uint8_t hashA[64],hashB[64];
	
    //--- luffa7	
	luffaHash(hash,hashA);

#ifdef DUMP_INFO_PERSTEP
	 printf("-----------------luffa output-------------------\n");
	 pByte=(unsigned char *)hashA;
	 BinToHexStr(strInfo,pByte,64);  
	 printf("hashA=%s\n",strInfo);
#endif

	//---cubehash---    
	cubeHash(hashA,hashB);

#ifdef DUMP_INFO_PERSTEP
	 printf("-----------------cubehash output-------------------\n");
	 pByte=(unsigned char *)hashB;
	 BinToHexStr(strInfo,pByte,64);
	 printf("hashB=%s\n",strInfo);
#endif

	//---shavite---
	sph_shavite512_context  shavite1;
	sph_shavite512_init(&shavite1);
    sph_shavite512 (&shavite1, hashB, 64);
    sph_shavite512_close(&shavite1, hashA);

#ifdef DUMP_INFO_PERSTEP
	 printf("-----------------shavite output-------------------\n");
	 pByte=(unsigned char *)hashA;
	 BinToHexStr(strInfo,pByte,64);  
	 printf("hashA=%s\n",strInfo);
#endif

    //-------simd512 vect128 --------------	
	simdHash(hashA,hashB);

#ifdef DUMP_INFO_PERSTEP
	 printf("-----------------simd512 vect128 output-------------------\n");
	 pByte=(unsigned char *)hashB;
	 BinToHexStr(strInfo,pByte,64);
	 printf("hashB=%s\n",strInfo);
#endif

	//---echo---
	echoHash(hashB,hashA);

#ifdef DUMP_INFO_PERSTEP
	 printf("-----------------echo output-------------------\n");
	 pByte=(unsigned char *)hashA;
	 BinToHexStr(strInfo,pByte,64);  
	 printf("hashA=%s\n",strInfo);

	 printf("-----------------END: the half of hashA is final result-------------------\n");
	 pByte=(unsigned char *)hashA;
	 BinToHexStr(strInfo,pByte,32);
	 printf("hash64=%s\n",strInfo);
#endif

    memcpy(state, hashA, 32);
}

extern FILE* g_logwork_file;

void x11_log_work(uint32_t *endiandata,uint32_t nonce,uint8_t *hash64)
{
	uint8_t work_str[80] = {0};
	uint8_t nonce_str[4] = {0};
	uint8_t hash_str[32];
	uint8_t msg[256] = {0};
	uint8_t *pwork = (uint8_t *)endiandata;
	uint8_t *pnonce = (uint8_t*)&nonce;
	uint8_t *phash = (uint8_t *)hash64;
	BinToHexStr(work_str,pwork, 80);
	BinToHexStr(nonce_str, pnonce, 4);
	BinToHexStr(hash_str,phash,32);
	sprintf(msg, "work %s nonce %s hash %s",work_str,nonce_str,hash_str);
	printf(" pattern %s\n",msg);
	if(g_logwork_file){
		fwrite(msg, strlen(msg), 1, g_logwork_file);
		fwrite("\n", 1, 1, g_logwork_file);
		fflush(g_logwork_file);
	}
}

uint32_t max_nonce = 0xffff0000;
void x11_hash(uint8_t *work)
{
	int k;
	uint32_t endiandata[32],hash_step = 0;;
	uint8_t hash64[64];
	
	unsigned char target[32] = {0};
	uint32_t *htarg = (uint32_t *)target;
	uint32_t *pnonce = (uint32_t *)work;
	htarg[7] = 0xff;
    for (k=0; k < 20; k++){
        be32enc(&endiandata[k], ((uint32_t*)work)[k]);
    }
	while(hash_step < max_nonce){
		pnonce[19] = ++hash_step;
			
		be32enc(&endiandata[19], hash_step); 
		Xhash(hash64, endiandata);
		if(fulltest(hash64,target)){
			x11_log_work(endiandata,hash_step,hash64);
		}

	}
}

void doOneTestData(int testIndex)
{
	int k;
	bool check_ret;
	FILE *fd;
	uint32_t endiandata[32];
	uint32_t pdata[32];
	uint32_t hash64_ret[8] __attribute__((aligned(32)));
	uint32_t hash64[8] __attribute__((aligned(32)));
	char outFile[256];
	
	sprintf(outFile,"C:\\MinGW\\msys\\1.0\\home\\Administrator\\testdata\\workdata%d.bin",testIndex);
	fd=fopen(outFile,"rb");
	if(!fd)
	{
		printf("read failed on %s\n",outFile);
		return;
	}
	
	fread(pdata,1,32*sizeof(uint32_t),fd);
	fclose(fd);

#ifdef DUMP_INFO_PERSTEP
	char strInfo[1024];
	unsigned char *pByte;
	
	printf("-----------------work.data-------------------\n");
	pByte=(unsigned char *)pdata;
	BinToHexStr(strInfo,pByte,128);  // 32*4 = 128 bytes
	printf("work.data=%s\n",strInfo);
#endif

    for (k=0; k < 32; k++)
    {
        be32enc(&endiandata[k], ((uint32_t*)pdata)[k]);
    }

	Xhash(hash64, endiandata);

	sprintf(outFile,"C:\\MinGW\\msys\\1.0\\home\\Administrator\\testdata\\resulthash%d.bin",testIndex);
	fd=fopen(outFile,"rb");
	if(!fd)
	{
		printf("read failed on %s\n",outFile);
		return;
	}
	
	fread(hash64_ret,1,8*sizeof(uint32_t),fd);
	fclose(fd);

	check_ret=true;
	for(k=0;k<8;k++)
	{
		if(hash64_ret[k]!=hash64[k])
			check_ret=false;
	}

	if(check_ret)
		printf("test file workdata%d.bin resulthash%d.bin is OK!\n",testIndex,testIndex);
	else printf("test file workdata%d.bin resulthash%d.bin is FAILED!\n",testIndex,testIndex);
}

void doOneGoldenData(int testIndex)
{
	int k;
	bool check_ret;
	FILE *fd;
	unsigned char target[32];
	uint32_t endiandata[32];
	uint32_t pdata[32];
	uint8_t datatmp[128];
	uint32_t hash64_ret[8] __attribute__((aligned(32)));
	uint32_t hash64[8] __attribute__((aligned(32)));
	char outFile[256];
	
	sprintf(outFile,"pattern/golden/1/workdata%d.bin",testIndex);
	fd=fopen(outFile,"rb");
//	fread(pdata,1,32*sizeof(uint32_t),fd);
	
	fread(datatmp,1,128,fd);
	fclose(fd);
	memcpy((void*)pdata,datatmp,128);
#ifdef DUMP_INFO_PERSTEP
	char strInfo[1024];
	unsigned char *pByte;
	printf("-----------------work.data-------------------\n");
	pByte=(unsigned char *)pdata;
	BinToHexStr(strInfo,pByte,128);  // 32*4 = 128 bytes
	printf("work.data=%s\n",strInfo);
	
	BinToHexStr(strInfo,datatmp,128);  // 32*4 = 128 bytes	
	printf("work.data=%s\n",strInfo);
#endif

    for (k=0; k < 32; k++)
    {
        be32enc(&endiandata[k], ((uint32_t*)pdata)[k]);
    }

	Xhash(hash64, endiandata);

	sprintf(outFile,"pattern/golden/1/resulthash%d.bin",testIndex);
	fd=fopen(outFile,"rb");
	fread(hash64_ret,1,8*sizeof(uint32_t),fd);
	fclose(fd);

	check_ret=true;
	for(k=0;k<8;k++)
	{
		if(hash64_ret[k]!=hash64[k])
			check_ret=false;
	}

	if(check_ret)
		printf("test file workdata%d.bin resulthash%d.bin is OK!\n",testIndex,testIndex);
	else printf("test file workdata%d.bin resulthash%d.bin is FAILED!\n",testIndex,testIndex);

	sprintf(outFile,"pattern/golden/1/target%d.bin",testIndex);
	fd=fopen(outFile,"rb");
	fread(target,1,8*sizeof(uint32_t),fd);
	fclose(fd);
	
	if (fulltest(hash64, target)) 
        printf("This is golden nonce!\n");
	else printf("This is NOT golden nonce!\n");
}

bool dohash_X11(uint32_t *pworkdata, const uint32_t *pworktarget, unsigned char *outHash)
{
    uint32_t hash64[8] __attribute__((aligned(32)));
    uint32_t endiandata[32];
    
    int kk=0;
    for (; kk < 32; kk++)
    {
        be32enc(&endiandata[kk], ((uint32_t*)pworkdata)[kk]);
    };

    Xhash(hash64, endiandata);
    if (fulltest(hash64, pworktarget)) 
	{
        memcpy(outHash,hash64,32);
        return true;
    }
    
    return false;
}

