

/* this ALWAYS GENERATED file contains the IIDs and CLSIDs */

/* link this file in with the server and any clients */


 /* File created by MIDL compiler version 7.00.0555 */
/* at Tue Mar 26 13:41:22 2013
 */
/* Compiler settings for interop\DeckLinkAPI.idl:
    Oicf, W1, Zp8, env=Win32 (32b run), target_arch=X86 7.00.0555 
    protocol : dce , ms_ext, c_ext, robust
    error checks: allocation ref bounds_check enum stub_data 
    VC __declspec() decoration level: 
         __declspec(uuid()), __declspec(selectany), __declspec(novtable)
         DECLSPEC_UUID(), MIDL_INTERFACE()
*/
/* @@MIDL_FILE_HEADING(  ) */

#pragma warning( disable: 4049 )  /* more than 64k source lines */


#ifdef __cplusplus
extern "C"{
#endif 


#include <rpc.h>
#include <rpcndr.h>

#ifdef _MIDL_USE_GUIDDEF_

#ifndef INITGUID
#define INITGUID
#include <guiddef.h>
#undef INITGUID
#else
#include <guiddef.h>
#endif

#define MIDL_DEFINE_GUID(type,name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
        DEFINE_GUID(name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8)

#else // !_MIDL_USE_GUIDDEF_

#ifndef __IID_DEFINED__
#define __IID_DEFINED__

typedef struct _IID
{
    unsigned long x;
    unsigned short s1;
    unsigned short s2;
    unsigned char  c[8];
} IID;

#endif // __IID_DEFINED__

#ifndef CLSID_DEFINED
#define CLSID_DEFINED
typedef IID CLSID;
#endif // CLSID_DEFINED

#define MIDL_DEFINE_GUID(type,name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
        const type name = {l,w1,w2,{b1,b2,b3,b4,b5,b6,b7,b8}}

#endif !_MIDL_USE_GUIDDEF_

MIDL_DEFINE_GUID(IID, LIBID_DeckLinkAPI,0xD864517A,0xEDD5,0x466D,0x86,0x7D,0xC8,0x19,0xF1,0xC0,0x52,0xBB);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoOutputCallback,0x20AA5225,0x1958,0x47CB,0x82,0x0B,0x80,0xA8,0xD5,0x21,0xA6,0xEE);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkInputCallback,0xDD04E5EC,0x7415,0x42AB,0xAE,0x4A,0xE8,0x0C,0x4D,0xFC,0x04,0x4A);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkMemoryAllocator,0xB36EB6E7,0x9D29,0x4AA8,0x92,0xEF,0x84,0x3B,0x87,0xA2,0x89,0xE8);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkAudioOutputCallback,0x403C681B,0x7F46,0x4A12,0xB9,0x93,0x2B,0xB1,0x27,0x08,0x4E,0xE6);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkIterator,0x74E936FC,0xCC28,0x4A67,0x81,0xA0,0x1E,0x94,0xE5,0x2D,0x4E,0x69);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkAPIInformation,0x7BEA3C68,0x730D,0x4322,0xAF,0x34,0x8A,0x71,0x52,0xB5,0x32,0xA4);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDisplayModeIterator,0x9C88499F,0xF601,0x4021,0xB8,0x0B,0x03,0x2E,0x4E,0xB4,0x1C,0x35);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDisplayMode,0x3EB2C1AB,0x0A3D,0x4523,0xA3,0xAD,0xF4,0x0D,0x7F,0xB1,0x4E,0x78);


MIDL_DEFINE_GUID(IID, IID_IDeckLink,0x62BFF75D,0x6569,0x4E55,0x8D,0x4D,0x66,0xAA,0x03,0x82,0x9A,0xBC);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkOutput,0xA3EF0963,0x0862,0x44ED,0x92,0xA9,0xEE,0x89,0xAB,0xF4,0x31,0xC7);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkInput,0x6D40EF78,0x28B9,0x4E21,0x99,0x0D,0x95,0xBB,0x77,0x50,0xA0,0x4F);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkTimecode,0xBC6CFBD3,0x8317,0x4325,0xAC,0x1C,0x12,0x16,0x39,0x1E,0x93,0x40);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoFrame,0x3F716FE0,0xF023,0x4111,0xBE,0x5D,0xEF,0x44,0x14,0xC0,0x5B,0x17);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkMutableVideoFrame,0x69E2639F,0x40DA,0x4E19,0xB6,0xF2,0x20,0xAC,0xE8,0x15,0xC3,0x90);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoFrame3DExtensions,0xDA0F7E4A,0xEDC7,0x48A8,0x9C,0xDD,0x2D,0xB5,0x1C,0x72,0x9C,0xD7);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoInputFrame,0x05CFE374,0x537C,0x4094,0x9A,0x57,0x68,0x05,0x25,0x11,0x8F,0x44);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoFrameAncillary,0x732E723C,0xD1A4,0x4E29,0x9E,0x8E,0x4A,0x88,0x79,0x7A,0x00,0x04);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkAudioInputPacket,0xE43D5870,0x2894,0x11DE,0x8C,0x30,0x08,0x00,0x20,0x0C,0x9A,0x66);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkScreenPreviewCallback,0xB1D3F49A,0x85FE,0x4C5D,0x95,0xC8,0x0B,0x5D,0x5D,0xCC,0xD4,0x38);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkGLScreenPreviewHelper,0x504E2209,0xCAC7,0x4C1A,0x9F,0xB4,0xC5,0xBB,0x62,0x74,0xD2,0x2F);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkConfiguration,0xC679A35B,0x610C,0x4D09,0xB7,0x48,0x1D,0x04,0x78,0x10,0x0F,0xC0);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkAttributes,0xABC11843,0xD966,0x44CB,0x96,0xE2,0xA1,0xCB,0x5D,0x31,0x35,0xC4);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkKeyer,0x89AFCAF5,0x65F8,0x421E,0x98,0xF7,0x96,0xFE,0x5F,0x5B,0xFB,0xA3);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoConversion,0x3BBCB8A2,0xDA2C,0x42D9,0xB5,0xD8,0x88,0x08,0x36,0x44,0xE9,0x9A);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDeckControlStatusCallback,0xE5F693C1,0x4283,0x4716,0xB1,0x8F,0xC1,0x43,0x15,0x21,0x95,0x5B);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDeckControl,0x522A9E39,0x0F3C,0x4742,0x94,0xEE,0xD8,0x0D,0xE3,0x35,0xDA,0x1D);


MIDL_DEFINE_GUID(CLSID, CLSID_CDeckLinkIterator,0xD9EDA3B3,0x2887,0x41FA,0xB7,0x24,0x01,0x7C,0xF1,0xEB,0x1D,0x37);


MIDL_DEFINE_GUID(CLSID, CLSID_CDeckLinkAPIInformation,0x263CA19F,0xED09,0x482E,0x9F,0x9D,0x84,0x00,0x57,0x83,0xA2,0x37);


MIDL_DEFINE_GUID(CLSID, CLSID_CDeckLinkGLScreenPreviewHelper,0xF63E77C7,0xB655,0x4A4A,0x9A,0xD0,0x3C,0xA8,0x5D,0x39,0x43,0x43);


MIDL_DEFINE_GUID(CLSID, CLSID_CDeckLinkVideoConversion,0x7DBBBB11,0x5B7B,0x467D,0xAE,0xA4,0xCE,0xA4,0x68,0xFD,0x36,0x8C);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDeckControl_v7_9,0xA4D81043,0x0619,0x42B7,0x8E,0xD6,0x60,0x2D,0x29,0x04,0x1D,0xF7);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDisplayModeIterator_v7_6,0x455D741F,0x1779,0x4800,0x86,0xF5,0x0B,0x5D,0x13,0xD7,0x97,0x51);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDisplayMode_v7_6,0x87451E84,0x2B7E,0x439E,0xA6,0x29,0x43,0x93,0xEA,0x4A,0x85,0x50);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkOutput_v7_6,0x29228142,0xEB8C,0x4141,0xA6,0x21,0xF7,0x40,0x26,0x45,0x09,0x55);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkInput_v7_6,0x300C135A,0x9F43,0x48E2,0x99,0x06,0x6D,0x79,0x11,0xD9,0x3C,0xF1);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkTimecode_v7_6,0xEFB9BCA6,0xA521,0x44F7,0xBD,0x69,0x23,0x32,0xF2,0x4D,0x9E,0xE6);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoFrame_v7_6,0xA8D8238E,0x6B18,0x4196,0x99,0xE1,0x5A,0xF7,0x17,0xB8,0x3D,0x32);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkMutableVideoFrame_v7_6,0x46FCEE00,0xB4E6,0x43D0,0x91,0xC0,0x02,0x3A,0x7F,0xCE,0xB3,0x4F);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoInputFrame_v7_6,0x9A74FA41,0xAE9F,0x47AC,0x8C,0xF4,0x01,0xF4,0x2D,0xD5,0x99,0x65);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkScreenPreviewCallback_v7_6,0x373F499D,0x4B4D,0x4518,0xAD,0x22,0x63,0x54,0xE5,0xA5,0x82,0x5E);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkGLScreenPreviewHelper_v7_6,0xBA575CD9,0xA15E,0x497B,0xB2,0xC2,0xF9,0xAF,0xE7,0xBE,0x4E,0xBA);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoConversion_v7_6,0x3EB504C9,0xF97D,0x40FE,0xA1,0x58,0xD4,0x07,0xD4,0x8C,0xB5,0x3B);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkConfiguration_v7_6,0xB8EAD569,0xB764,0x47F0,0xA7,0x3F,0xAE,0x40,0xDF,0x6C,0xBF,0x10);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoOutputCallback_v7_6,0xE763A626,0x4A3C,0x49D1,0xBF,0x13,0xE7,0xAD,0x36,0x92,0xAE,0x52);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkInputCallback_v7_6,0x31D28EE7,0x88B6,0x4CB1,0x89,0x7A,0xCD,0xBF,0x79,0xA2,0x64,0x14);


MIDL_DEFINE_GUID(CLSID, CLSID_CDeckLinkGLScreenPreviewHelper_v7_6,0xD398CEE7,0x4434,0x4CA3,0x9B,0xA6,0x5A,0xE3,0x45,0x56,0xB9,0x05);


MIDL_DEFINE_GUID(CLSID, CLSID_CDeckLinkVideoConversion_v7_6,0xFFA84F77,0x73BE,0x4FB7,0xB0,0x3E,0xB5,0xE4,0x4B,0x9F,0x75,0x9B);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkInputCallback_v7_3,0xFD6F311D,0x4D00,0x444B,0x9E,0xD4,0x1F,0x25,0xB5,0x73,0x0A,0xD0);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkOutput_v7_3,0x271C65E3,0xC323,0x4344,0xA3,0x0F,0xD9,0x08,0xBC,0xB2,0x0A,0xA3);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkInput_v7_3,0x4973F012,0x9925,0x458C,0x87,0x1C,0x18,0x77,0x4C,0xDB,0xBE,0xCB);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoInputFrame_v7_3,0xCF317790,0x2894,0x11DE,0x8C,0x30,0x08,0x00,0x20,0x0C,0x9A,0x66);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDisplayModeIterator_v7_1,0xB28131B6,0x59AC,0x4857,0xB5,0xAC,0xCD,0x75,0xD5,0x88,0x3E,0x2F);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkDisplayMode_v7_1,0xAF0CD6D5,0x8376,0x435E,0x84,0x33,0x54,0xF9,0xDD,0x53,0x0A,0xC3);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoFrame_v7_1,0x333F3A10,0x8C2D,0x43CF,0xB7,0x9D,0x46,0x56,0x0F,0xEE,0xA1,0xCE);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoInputFrame_v7_1,0xC8B41D95,0x8848,0x40EE,0x9B,0x37,0x6E,0x34,0x17,0xFB,0x11,0x4B);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkAudioInputPacket_v7_1,0xC86DE4F6,0xA29F,0x42E3,0xAB,0x3A,0x13,0x63,0xE2,0x9F,0x07,0x88);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkVideoOutputCallback_v7_1,0xEBD01AFA,0xE4B0,0x49C6,0xA0,0x1D,0xED,0xB9,0xD1,0xB5,0x5F,0xD9);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkInputCallback_v7_1,0x7F94F328,0x5ED4,0x4E9F,0x97,0x29,0x76,0xA8,0x6B,0xDC,0x99,0xCC);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkOutput_v7_1,0xAE5B3E9B,0x4E1E,0x4535,0xB6,0xE8,0x48,0x0F,0xF5,0x2F,0x6C,0xE5);


MIDL_DEFINE_GUID(IID, IID_IDeckLinkInput_v7_1,0x2B54EDEF,0x5B32,0x429F,0xBA,0x11,0xBB,0x99,0x05,0x96,0xEA,0xCD);

#undef MIDL_DEFINE_GUID

#ifdef __cplusplus
}
#endif



