//=============================================================================
#include 	"main.h"

//=============================================================================
s16 const hSinCosTable[1024]={ 
	0, 0x64, 0xC9, 0x12D, 0x192, 0x1F7, 0x25B, 0x2C0,
	0x324, 0x389, 0x3EE, 0x452, 0x4B7, 0x51B, 0x580, 0x5E4,
	0x649, 0x6AD, 0x712, 0x776, 0x7DB, 0x83F, 0x8A4, 0x908, 
	0x96C, 0x9D1, 0xA35, 0xA99, 0xAFE, 0xB62, 0xBC6, 0xC2A,
	0xC8E, 0xCF2, 0xD57, 0xDBB, 0xE1F, 0xE83, 0xEE7, 0xF4B,
	0xFAE, 0x1012, 0x1076, 0x10DA, 0x113E, 0x11A1, 0x1205, 0x1269,
	0x12CC, 0x1330, 0x1393, 0x13F6, 0x145A, 0x14BD, 0x1520, 0x1584,
	0x15E7, 0x164A, 0x16AD, 0x1710, 0x1773, 0x17D6, 0x1839, 0x189B,
	0x18FE, 0x1961, 0x19C3, 0x1A26, 0x1A88, 0x1AEB, 0x1B4D, 0x1BAF,
	0x1C12, 0x1C74, 0x1CD6, 0x1D38, 0x1D9A, 0x1DFC, 0x1E5D, 0x1EBF,
	0x1F21, 0x1F82, 0x1FE4, 0x2045, 0x20A7, 0x2108, 0x2169, 0x21CA,
	0x222B, 0x228C, 0x22ED, 0x234E, 0x23AE, 0x240F, 0x2470, 0x24D0,
	0x2530, 0x2591, 0x25F1, 0x2651, 0x26B1, 0x2711, 0x2770, 0x27D0,
	0x2830, 0x288F, 0x28EE, 0x294E, 0x29AD, 0x2A0C, 0x2A6B, 0x2ACA,
	0x2B29, 0x2B87, 0x2BE6, 0x2C44, 0x2CA3, 0x2D01, 0x2D5F, 0x2DBD,
	0x2E1B, 0x2E79, 0x2ED7, 0x2F34, 0x2F92, 0x2FEF, 0x304C, 0x30A9,
	0x3107, 0x3163, 0x31C0, 0x321D, 0x3279, 0x32D6, 0x3332, 0x338E,
	0x33EA, 0x3446, 0x34A2, 0x34FE, 0x3559, 0x35B5, 0x3610, 0x366B,
	0x36C6, 0x3721, 0x377C, 0x37D6, 0x3831, 0x388B, 0x38E5, 0x393F,
	0x3999, 0x39F3, 0x3A4D, 0x3AA6, 0x3B00, 0x3B59, 0x3BB2, 0x3C0B,
	0x3C64, 0x3CBC, 0x3D15, 0x3D6D, 0x3DC5, 0x3E1D, 0x3E75, 0x3ECD,
	0x3F25, 0x3F7C, 0x3FD3, 0x402B, 0x4082, 0x40D8, 0x412F, 0x4186,
	0x41DC, 0x4232, 0x4288, 0x42DE, 0x4334, 0x4389, 0x43DF, 0x4434,
	0x4489, 0x44DE, 0x4533, 0x4587, 0x45DC, 0x4630, 0x4684, 0x46D8,
	0x472C, 0x477F, 0x47D2, 0x4826, 0x4879, 0x48CC, 0x491E, 0x4971,
	0x49C3, 0x4A15, 0x4A67, 0x4AB9, 0x4B0B, 0x4B5C, 0x4BAD, 0x4BFE,
	0x4C4F, 0x4CA0, 0x4CF0, 0x4D41, 0x4D91, 0x4DE1, 0x4E31, 0x4E80,
	0x4ED0, 0x4F1F, 0x4F6E, 0x4FBD, 0x500B, 0x505A, 0x50A8, 0x50F6,
	0x5144, 0x5191, 0x51DF, 0x522C, 0x5279, 0x52C6, 0x5313, 0x535F,
	0x53AB, 0x53F7, 0x5443, 0x548F, 0x54DA, 0x5525, 0x5571, 0x55BB,
	0x5606, 0x5650, 0x569B, 0x56E5, 0x572E, 0x5778, 0x57C1, 0x580A,
	0x5853, 0x589C, 0x58E5, 0x592D, 0x5975, 0x59BD, 0x5A04, 0x5A4C,
	0x5A93, 0x5ADA, 0x5B21, 0x5B67, 0x5BAE, 0x5BF4, 0x5C3A, 0x5C7F,
	0x5CC5, 0x5D0A, 0x5D4F, 0x5D94, 0x5DD8, 0x5E1D, 0x5E61, 0x5EA5,
	0x5EE8, 0x5F2C, 0x5F6F, 0x5FB2, 0x5FF4, 0x6037, 0x6079, 0x60BB,
	0x60FD, 0x613E, 0x6180, 0x61C1, 0x6202, 0x6242, 0x6283, 0x62C3,
	0x6303, 0x6342, 0x6382, 0x63C1, 0x6400, 0x643F, 0x647D, 0x64BB,
	0x64F9, 0x6537, 0x6574, 0x65B2, 0x65EF, 0x662B, 0x6668, 0x66A4,
	0x66E0, 0x671C, 0x6757, 0x6792, 0x67CD, 0x6808, 0x6843, 0x687D,
	0x68B7, 0x68F1, 0x692A, 0x6963, 0x699C, 0x69D5, 0x6A0E, 0x6A46,
	0x6A7E, 0x6AB5, 0x6AED, 0x6B24, 0x6B5B, 0x6B92, 0x6BC8, 0x6BFE,
	0x6C34, 0x6C6A, 0x6C9F, 0x6CD4, 0x6D09, 0x6D3E, 0x6D72, 0x6DA6,
	0x6DDA, 0x6E0D, 0x6E40, 0x6E73, 0x6EA6, 0x6ED9, 0x6F0B, 0x6F3D,
	0x6F6E, 0x6FA0, 0x6FD1, 0x7002, 0x7032, 0x7062, 0x7092, 0x70C2,
	0x70F2, 0x7121, 0x7150, 0x717E, 0x71AD, 0x71DB, 0x7209, 0x7236,
	0x7264, 0x7291, 0x72BD, 0x72EA, 0x7316, 0x7342, 0x736E, 0x7399,
	0x73C4, 0x73EF, 0x7419, 0x7443, 0x746D, 0x7497, 0x74C0, 0x74EA,
	0x7512, 0x753B, 0x7563, 0x758B, 0x75B3, 0x75DA, 0x7601, 0x7628,
	0x764F, 0x7675, 0x769B, 0x76C1, 0x76E6, 0x770B, 0x7730, 0x7754,
	0x7779, 0x779D, 0x77C0, 0x77E4, 0x7807, 0x782A, 0x784C, 0x786E,
	0x7890, 0x78B2, 0x78D3, 0x78F4, 0x7915, 0x7936, 0x7956, 0x7976,
	0x7995, 0x79B5, 0x79D4, 0x79F2, 0x7A11, 0x7A2F, 0x7A4D, 0x7A6A,
	0x7A87, 0x7AA4, 0x7AC1, 0x7ADD, 0x7AF9, 0x7B15, 0x7B31, 0x7B4C,
	0x7B67, 0x7B81, 0x7B9B, 0x7BB5, 0x7BCF, 0x7BE8, 0x7C02, 0x7C1A,
	0x7C33, 0x7C4B, 0x7C63, 0x7C7A, 0x7C92, 0x7CA9, 0x7CBF, 0x7CD6,
	0x7CEC, 0x7D02, 0x7D17, 0x7D2C, 0x7D41, 0x7D56, 0x7D6A, 0x7D7E,
	0x7D91, 0x7DA5, 0x7DB8, 0x7DCB, 0x7DDD, 0x7DEF, 0x7E01, 0x7E13,
	0x7E24, 0x7E35, 0x7E45, 0x7E56, 0x7E66, 0x7E75, 0x7E85, 0x7E94,
	0x7EA3, 0x7EB1, 0x7EBF, 0x7ECD, 0x7EDB, 0x7EE8, 0x7EF5, 0x7F01,
	0x7F0E, 0x7F1A, 0x7F25, 0x7F31, 0x7F3C, 0x7F47, 0x7F51, 0x7F5B,
	0x7F65, 0x7F6F, 0x7F78, 0x7F81, 0x7F8A, 0x7F92, 0x7F9A, 0x7FA2,
	0x7FA9, 0x7FB0, 0x7FB7, 0x7FBE, 0x7FC4, 0x7FCA, 0x7FCF, 0x7FD4,
	0x7FD9, 0x7FDE, 0x7FE2, 0x7FE6, 0x7FEA, 0x7FED, 0x7FF1, 0x7FF3,
	0x7FF6, 0x7FF8, 0x7FFA, 0x7FFB, 0x7FFD, 0x7FFE, 0x7FFE, 0x7FFE,
	0x7FFE, 0x7FFE, 0x7FFE, 0x7FFD, 0x7FFB, 0x7FFA, 0x7FF8, 0x7FF6,
	0x7FF3, 0x7FF1, 0x7FED, 0x7FEA, 0x7FE6, 0x7FE2, 0x7FDE, 0x7FD9,
	0x7FD4, 0x7FCF, 0x7FCA, 0x7FC4, 0x7FBE, 0x7FB7, 0x7FB0, 0x7FA9,
	0x7FA2, 0x7F9A, 0x7F92, 0x7F8A, 0x7F81, 0x7F78, 0x7F6F, 0x7F65,
	0x7F5B, 0x7F51, 0x7F47, 0x7F3C, 0x7F31, 0x7F25, 0x7F1A, 0x7F0E,
	0x7F01, 0x7EF5, 0x7EE8, 0x7EDB, 0x7ECD, 0x7EBF, 0x7EB1, 0x7EA3,
	0x7E94, 0x7E85, 0x7E75, 0x7E66, 0x7E56, 0x7E45, 0x7E35, 0x7E24,
	0x7E13, 0x7E01, 0x7DEF, 0x7DDD, 0x7DCB, 0x7DB8, 0x7DA5, 0x7D91,
	0x7D7E, 0x7D6A, 0x7D56, 0x7D41, 0x7D2C, 0x7D17, 0x7D02, 0x7CEC,
	0x7CD6, 0x7CBF, 0x7CA9, 0x7C92, 0x7C7A, 0x7C63, 0x7C4B, 0x7C33,
	0x7C1A, 0x7C02, 0x7BE8, 0x7BCF, 0x7BB5, 0x7B9B, 0x7B81, 0x7B67,
	0x7B4C, 0x7B31, 0x7B15, 0x7AF9, 0x7ADD, 0x7AC1, 0x7AA4, 0x7A87,
	0x7A6A, 0x7A4D, 0x7A2F, 0x7A11, 0x79F2, 0x79D4, 0x79B5, 0x7995,
	0x7976, 0x7956, 0x7936, 0x7915, 0x78F4, 0x78D3, 0x78B2, 0x7890,
	0x786E, 0x784C, 0x782A, 0x7807, 0x77E4, 0x77C0, 0x779D, 0x7779,
	0x7754, 0x7730, 0x770B, 0x76E6, 0x76C1, 0x769B, 0x7675, 0x764F,
	0x7628, 0x7601, 0x75DA, 0x75B3, 0x758B, 0x7563, 0x753B, 0x7512,
	0x74EA, 0x74C0, 0x7497, 0x746D, 0x7443, 0x7419, 0x73EF, 0x73C4,
	0x7399, 0x736E, 0x7342, 0x7316, 0x72EA, 0x72BD, 0x7291, 0x7264,
	0x7236, 0x7209, 0x71DB, 0x71AD, 0x717E, 0x7150, 0x7121, 0x70F2,
	0x70C2, 0x7092, 0x7062, 0x7032, 0x7002, 0x6FD1, 0x6FA0, 0x6F6E,
	0x6F3D, 0x6F0B, 0x6ED9, 0x6EA6, 0x6E73, 0x6E40, 0x6E0D, 0x6DDA,
	0x6DA6, 0x6D72, 0x6D3E, 0x6D09, 0x6CD4, 0x6C9F, 0x6C6A, 0x6C34,
	0x6BFE, 0x6BC8, 0x6B92, 0x6B5B, 0x6B24, 0x6AED, 0x6AB5, 0x6A7E,
	0x6A46, 0x6A0E, 0x69D5, 0x699C, 0x6963, 0x692A, 0x68F1, 0x68B7,
	0x687D, 0x6843, 0x6808, 0x67CD, 0x6792, 0x6757, 0x671C, 0x66E0,
	0x66A4, 0x6668, 0x662B, 0x65EF, 0x65B2, 0x6574, 0x6537, 0x64F9,
	0x64BB, 0x647D, 0x643F, 0x6400, 0x63C1, 0x6382, 0x6342, 0x6303,
	0x62C3, 0x6283, 0x6242, 0x6202, 0x61C1, 0x6180, 0x613E, 0x60FD,
	0x60BB, 0x6079, 0x6037, 0x5FF4, 0x5FB2, 0x5F6F, 0x5F2C, 0x5EE8,
	0x5EA5, 0x5E61, 0x5E1D, 0x5DD8, 0x5D94, 0x5D4F, 0x5D0A, 0x5CC5,
	0x5C7F, 0x5C3A, 0x5BF4, 0x5BAE, 0x5B67, 0x5B21, 0x5ADA, 0x5A93,
	0x5A4C, 0x5A04, 0x59BD, 0x5975, 0x592D, 0x58E5, 0x589C, 0x5853,
	0x580A, 0x57C1, 0x5778, 0x572E, 0x56E5, 0x569B, 0x5650, 0x5606,
	0x55BB, 0x5571, 0x5525, 0x54DA, 0x548F, 0x5443, 0x53F7, 0x53AB,
	0x535F, 0x5313, 0x52C6, 0x5279, 0x522C, 0x51DF, 0x5191, 0x5144,
	0x50F6, 0x50A8, 0x505A, 0x500B, 0x4FBD, 0x4F6E, 0x4F1F, 0x4ED0,
	0x4E80, 0x4E31, 0x4DE1, 0x4D91, 0x4D41, 0x4CF0, 0x4CA0, 0x4C4F,
	0x4BFE, 0x4BAD, 0x4B5C, 0x4B0B, 0x4AB9, 0x4A67, 0x4A15, 0x49C3,
	0x4971, 0x491E, 0x48CC, 0x4879, 0x4826, 0x47D2, 0x477F, 0x472C,
	0x46D8, 0x4684, 0x4630, 0x45DC, 0x4587, 0x4533, 0x44DE, 0x4489,
	0x4434, 0x43DF, 0x4389, 0x4334, 0x42DE, 0x4288, 0x4232, 0x41DC,
	0x4186, 0x412F, 0x40D8, 0x4082, 0x402B, 0x3FD3, 0x3F7C, 0x3F25,
	0x3ECD, 0x3E75, 0x3E1D, 0x3DC5, 0x3D6D, 0x3D15, 0x3CBC, 0x3C64,
	0x3C0B, 0x3BB2, 0x3B59, 0x3B00, 0x3AA6, 0x3A4D, 0x39F3, 0x3999,
	0x393F, 0x38E5, 0x388B, 0x3831, 0x37D6, 0x377C, 0x3721, 0x36C6,
	0x366B, 0x3610, 0x35B5, 0x3559, 0x34FE, 0x34A2, 0x3446, 0x33EA,
	0x338E, 0x3332, 0x32D6, 0x3279, 0x321D, 0x31C0, 0x3163, 0x3107,
	0x30A9, 0x304C, 0x2FEF, 0x2F92, 0x2F34, 0x2ED7, 0x2E79, 0x2E1B,
	0x2DBD, 0x2D5F, 0x2D01, 0x2CA3, 0x2C44, 0x2BE6, 0x2B87, 0x2B29,
	0x2ACA, 0x2A6B, 0x2A0C, 0x29AD, 0x294E, 0x28EE, 0x288F, 0x2830,
	0x27D0, 0x2770, 0x2711, 0x26B1, 0x2651, 0x25F1, 0x2591, 0x2530,
	0x24D0, 0x2470, 0x240F, 0x23AE, 0x234E, 0x22ED, 0x228C, 0x222B,
	0x21CA, 0x2169, 0x2108, 0x20A7, 0x2045, 0x1FE4, 0x1F82, 0x1F21,
	0x1EBF, 0x1E5D, 0x1DFC, 0x1D9A, 0x1D38, 0x1CD6, 0x1C74, 0x1C12,
	0x1BAF, 0x1B4D, 0x1AEB, 0x1A88, 0x1A26, 0x19C3, 0x1961, 0x18FE,
	0x189B, 0x1839, 0x17D6, 0x1773, 0x1710, 0x16AD, 0x164A, 0x15E7,
	0x1584, 0x1520, 0x14BD, 0x145A, 0x13F6, 0x1393, 0x1330, 0x12CC,
	0x1269, 0x1205, 0x11A1, 0x113E, 0x10DA, 0x1076, 0x1012, 0xFAE,
	0xF4B, 0xEE7, 0xE83, 0xE1F, 0xDBB, 0xD57, 0xCF2, 0xC8E,
	0xC2A, 0xBC6, 0xB62, 0xAFE, 0xA99, 0xA35, 0x9D1, 0x96C,
	0x908, 0x8A4, 0x83F, 0x7DB, 0x776, 0x712, 0x6AD, 0x649,
	0x5E4, 0x580, 0x51B, 0x4B7, 0x452, 0x3EE, 0x389, 0x324,
	0x2C0, 0x25B, 0x1F7, 0x192, 0x12D, 0xC9, 0x64, 0
};
//=============================================================================
s16 get_sin(u16 a1)
{
	if ( a1 >= 0x8000 )
		return -hSinCosTable[(a1 - 0x8000) >> 5];
	else
		return hSinCosTable[a1 >> 5];
}
//=============================================================================
s16 get_cos(u32 a1)
{
	return get_sin(a1 + 0x4000);
}
//=============================================================================
