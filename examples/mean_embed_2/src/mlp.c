#include "../include/mlp.h"

static const float phi_a_w0[OUTPUT0][PHI_A_H] = 
        {{-0.0712, -0.1064,  0.0829, -0.0613, -0.1034,  0.1828, -0.1816, -0.1680,
          0.2552, -0.0522,  0.1578,  0.2222,  0.2968,  0.0967, -0.1428, -0.1108,
         -0.4192,  0.2888, -0.2358, -0.0817, -0.0972, -0.1711,  0.0574, -0.2275},
        { 0.2312, -0.1308,  0.2763, -0.3347, -0.1600, -0.3166,  0.1986,  0.1450,
          0.0752, -0.3626, -0.1812,  0.0386,  0.0553,  0.1420,  0.1220,  0.2385,
          0.1138, -0.0805, -0.0999,  0.0641,  0.0791, -0.0325, -0.1671,  0.3535},
        { 0.3482,  0.0388,  0.1835, -0.0676, -0.2657, -0.0429,  0.0184, -0.2028,
          0.1248, -0.2311,  0.3759,  0.2991,  0.2532,  0.1218, -0.2372, -0.2879,
         -0.2200,  0.2978, -0.0530,  0.1220, -0.0325, -0.1115, -0.0511,  0.0037},
        {-0.1948, -0.2005, -0.1613, -0.1941,  0.1435, -0.2193,  0.0984, -0.3325,
         -0.0653,  0.0868,  0.1395, -0.3700, -0.0590,  0.2330, -0.1407,  0.2554,
          0.1562,  0.2448, -0.0649, -0.2819,  0.0528,  0.0479, -0.1232, -0.2559},
        {-0.3058,  0.0844, -0.2721,  0.2093,  0.1553, -0.1531, -0.3528,  0.0954,
          0.1921,  0.0520, -0.3780,  0.0343, -0.3395,  0.0299, -0.3348,  0.2086,
          0.1775,  0.2092, -0.2513, -0.3729,  0.1111,  0.0518, -0.2350, -0.1031},
        {-0.3280,  0.0304, -0.1232, -0.3138,  0.0150, -0.1558, -0.1337,  0.2085,
         -0.0798,  0.0611, -0.1580, -0.0429, -0.0222, -0.2424,  0.1159, -0.1876,
         -0.2442,  0.2854, -0.2007, -0.2001,  0.2215,  0.1065,  0.1947, -0.2235},
        {-0.3183, -0.0354, -0.1625, -0.2739,  0.2576,  0.0871, -0.0959, -0.1109,
         -0.0473, -0.0672,  0.0345,  0.2020, -0.2763,  0.0608, -0.3146,  0.2149,
          0.1724,  0.1308,  0.1112,  0.0560, -0.0192,  0.1338, -0.2540, -0.1687},
        {-0.2828, -0.2917,  0.0211, -0.2895,  0.1079,  0.0627, -0.1996, -0.1104,
         -0.2181, -0.2593,  0.0508,  0.0641, -0.1492,  0.1129, -0.1922,  0.1868,
         -0.0504,  0.2750, -0.2739, -0.0546,  0.2708,  0.1060,  0.0249, -0.0984},
        { 0.1434, -0.2773, -0.0919, -0.1473,  0.1378, -0.2877,  0.0709, -0.1354,
         -0.0957,  0.1281, -0.2401, -0.1320,  0.0311, -0.2665,  0.1775, -0.1598,
         -0.1272, -0.3746, -0.3200, -0.1464,  0.1128, -0.1672,  0.2830,  0.3378},
        {-0.0960, -0.1668,  0.0216, -0.0725, -0.3077, -0.3070, -0.2707,  0.2228,
          0.3534, -0.1011,  0.1424, -0.1142, -0.2399,  0.2151, -0.1116,  0.2720,
          0.0810, -0.1186, -0.1141,  0.0393,  0.2152, -0.0196, -0.3592, -0.0420},
        {-0.2989,  0.0974, -0.0752,  0.2167,  0.2403,  0.0860,  0.3203, -0.1122,
          0.2362,  0.0248,  0.2084,  0.1807, -0.2074,  0.3658, -0.2251,  0.1363,
          0.2378, -0.4014, -0.0138,  0.1672,  0.0964,  0.3507,  0.2466,  0.2364},
        { 0.0509,  0.2209,  0.2165, -0.1340, -0.1855, -0.2989, -0.1612,  0.0684,
         -0.3344, -0.0668, -0.2063,  0.2429,  0.0654,  0.0823,  0.1671, -0.1607,
          0.0412,  0.1792,  0.4476, -0.4720,  0.0696, -0.3830, -0.0732,  0.1058},
        { 0.2792,  0.2259,  0.0741, -0.1851,  0.1547,  0.0414, -0.1482, -0.0530,
         -0.1750,  0.2782, -0.2334, -0.0488,  0.2091,  0.1727, -0.3492,  0.1027,
         -0.1024,  0.1988,  0.3037, -0.0861,  0.2612,  0.3353,  0.5178,  0.1874},
        {-0.2915,  0.1257, -0.0879, -0.2291, -0.2712,  0.0894,  0.0048,  0.0468,
         -0.0638,  0.1943,  0.2690, -0.1886,  0.0456, -0.1509, -0.0105, -0.2355,
          0.1048, -0.2126, -0.1006, -0.2675, -0.1555, -0.2203,  0.2665, -0.2718},
        { 0.1799,  0.0644, -0.1061, -0.1791,  0.1467,  0.3779,  0.1611,  0.0735,
         -0.4474, -0.2905, -0.0619, -0.2220,  0.2301, -0.0976,  0.0798,  0.2212,
          0.0831,  0.3045,  0.2622, -0.0194, -0.2424, -0.0424,  0.5804,  0.1006},
        { 0.0828, -0.0055, -0.1092,  0.3157, -0.0458,  0.2426, -0.2037, -0.2936,
          0.2033,  0.2721,  0.0187, -0.3140, -0.1800, -0.3168,  0.0729,  0.0910,
         -0.1596, -0.1677,  0.1133,  0.0136,  0.0326,  0.0662, -0.1121, -0.2236},
        {-0.1296, -0.0080, -0.2178,  0.2457,  0.0725,  0.3180, -0.1449, -0.3226,
          0.1605,  0.2144,  0.2629,  0.1828,  0.2240,  0.0986,  0.1336, -0.0244,
         -0.2578,  0.1273, -0.6254, -0.1470,  0.1993, -0.1277,  0.2026, -0.1687},
        {-0.2050,  0.2352, -0.3164, -0.2756, -0.3283, -0.1105,  0.0261, -0.0186,
         -0.1072,  0.0438,  0.0390, -0.2885,  0.2076,  0.2931, -0.2257,  0.1590,
         -0.0656,  0.0989,  0.1377, -0.2657, -0.0067, -0.2681, -0.2660, -0.1745},
        { 0.2315, -0.1074,  0.0958, -0.3056, -0.1739,  0.0148,  0.0237, -0.2700,
         -0.1113,  0.0654, -0.3063, -0.0856,  0.0495,  0.2827,  0.0913, -0.1682,
         -0.0032, -0.0109,  0.3569, -0.0075, -0.0014,  0.2144,  0.4364,  0.1786},
        { 0.3324, -0.1227,  0.0230,  0.0726,  0.0831,  0.0845, -0.0187, -0.3424,
          0.5108,  0.1941, -0.1455,  0.0465,  0.1754,  0.0468, -0.2300, -0.2807,
          0.2723,  0.1669,  0.1417,  0.2825, -0.2002, -0.0971,  0.1336, -0.0616},
        { 0.2706,  0.0883, -0.1059, -0.1534, -0.1794,  0.1086,  0.2069,  0.0779,
         -0.1473,  0.0046,  0.1069, -0.0019,  0.0320, -0.4350,  0.0361,  0.3017,
          0.2567, -0.0609,  0.4020, -0.3362,  0.1702, -0.2900, -0.0054,  0.1079},
        { 0.2471, -0.1640,  0.3448,  0.0487, -0.0395, -0.2338, -0.1029,  0.0334,
          0.1258, -0.1725, -0.2195,  0.1711, -0.1671, -0.1898,  0.2942,  0.3279,
         -0.0009, -0.6206,  0.0455,  0.1578, -0.0771,  0.1008, -0.1259, -0.0761},
        {-0.0953,  0.1526,  0.4998,  0.1582,  0.3574, -0.1884, -0.0804, -0.1895,
          0.3223, -0.0182,  0.1742,  0.1203, -0.2689,  0.2590,  0.2118,  0.1036,
          0.1040, -0.4601, -0.2876,  0.0326,  0.3799,  0.2296,  0.1505,  0.0106},
        {-0.2505, -0.3978, -0.2441,  0.2073,  0.1664,  0.0420,  0.2092, -0.0649,
         -0.1656, -0.2019,  0.0421,  0.0787, -0.4329,  0.1727,  0.0292,  0.0533,
         -0.4603,  0.2112,  0.2096, -0.4242,  0.1540, -0.2893,  0.2056,  0.0827},
        { 0.2250,  0.0183, -0.0879,  0.1550, -0.2158, -0.2641, -0.1416, -0.0999,
          0.0886, -0.2420,  0.1847, -0.0487,  0.2623, -0.2057, -0.2274, -0.1562,
         -0.0188, -0.0452, -0.1617,  0.1200, -0.1980,  0.3523,  0.1085,  0.2039},
        {-0.2841, -0.0308, -0.3209, -0.0574, -0.0445, -0.1532, -0.1391,  0.0121,
         -0.0189,  0.2393,  0.2106, -0.2015, -0.0446, -0.2435, -0.3088, -0.2461,
         -0.1615, -0.0368,  0.0939, -0.2100, -0.3404,  0.3396, -0.1521, -0.2210},
        { 0.1976,  0.0829, -0.2833,  0.0031, -0.5334, -0.0234, -0.3062,  0.1773,
          0.2994, -0.1035,  0.0785,  0.1398, -0.0689,  0.0786, -0.1912,  0.0058,
          0.1308, -0.2160, -0.1852, -0.0266, -0.0044, -0.2206, -0.1520, -0.4545},
        {-0.2509, -0.0386,  0.0104, -0.0276,  0.1403, -0.2632, -0.1365,  0.0635,
         -0.3449, -0.3634, -0.1965,  0.0138,  0.0870,  0.2508, -0.1682,  0.0956,
          0.1881,  0.0100,  0.0503, -0.1770,  0.0773, -0.1674,  0.1090,  0.1298},
        { 0.0342, -0.2648, -0.0472,  0.1023,  0.0362,  0.1506,  0.2156,  0.1036,
          0.0771, -0.2933,  0.2129,  0.2955,  0.2300,  0.3041, -0.1129, -0.2748,
          0.0255,  0.0577,  0.0565, -0.0093,  0.3067, -0.0508, -0.0673,  0.1449},
        { 0.1143, -0.3537, -0.1388,  0.1410, -0.0894, -0.2456,  0.2090, -0.2197,
         -0.3265,  0.1082,  0.0740, -0.1193, -0.2859,  0.2902,  0.1274, -0.1502,
         -0.0144, -0.1391,  0.4043,  0.1775,  0.0081, -0.2247,  0.0119,  0.0472},
        { 0.2873,  0.0246, -0.1145, -0.1128, -0.1429, -0.3801,  0.0354,  0.3041,
          0.2344,  0.0782, -0.1879,  0.2963,  0.2024, -0.0992,  0.3164, -0.2065,
          0.0393, -0.2267, -0.4688, -0.1743, -0.2090, -0.0185, -0.0593, -0.1863},
        { 0.2873,  0.3362, -0.0536,  0.2923,  0.3511,  0.1803,  0.1132, -0.1624,
          0.3294, -0.1993, -0.1402, -0.2640, -0.2047,  0.0009, -0.3328, -0.1201,
         -0.0204,  0.3771, -0.1442,  0.0668,  0.0300, -0.2818,  0.2694,  0.2016}};



static const float phi_a_w1[PHI_A_V][OUTPUT0] =
       {{ 0.3329, -0.1276,  0.0918,  0.2119,  0.3917, -0.3107, -0.2685, -0.1820,
         -0.0174, -0.0038, -0.1845,  0.2183,  0.0061,  0.3297, -0.0361,  0.5149,
         -0.1369, -0.3979, -0.1449,  0.3987,  0.4457, -0.1420, -0.1927,  0.0442,
         -0.0847, -0.0561,  0.1585, -0.3072, -0.3367, -0.1182, -0.2269,  0.3008},
        { 0.3090, -0.0101, -0.3483,  0.0950, -0.1629,  0.0155, -0.0219, -0.3649,
         -0.1861,  0.0499,  0.3040, -0.0118, -0.3855,  0.2326, -0.4366,  0.4052,
         -0.0648, -0.2014, -0.4505,  0.3231,  0.1331,  0.1894,  0.2207, -0.1932,
         -0.2741, -0.3451,  0.1244,  0.0371, -0.2939, -0.3118,  0.1921,  0.1167},
        { 0.2376, -0.1729,  0.4389, -0.0996, -0.1011, -0.0809,  0.1965,  0.3883,
          0.0138,  0.3260, -0.4577, -0.3590, -0.2023, -0.0061, -0.4515,  0.0480,
          0.2738, -0.0590, -0.1110,  0.1829, -0.2011,  0.2585, -0.0424,  0.2681,
          0.3659,  0.3033,  0.2925,  0.0921, -0.0036, -0.2495,  0.3795, -0.0715},
        { 0.4243, -0.3684,  0.3214, -0.0147, -0.3961, -0.3262, -0.1087, -0.2791,
          0.3640,  0.3414,  0.0576,  0.0807,  0.2526, -0.3599, -0.0896, -0.3003,
          0.1847,  0.4085, -0.1963,  0.1871, -0.2922, -0.4090,  0.0260,  0.1939,
          0.1729,  0.0977, -0.0352, -0.0624,  0.1996, -0.2464, -0.3536,  0.2300}};




static const float phi_a_b0[OUTPUT0] = { 0.0354,  0.0430, -0.0494, -0.0428, -0.1345, -0.0336, -0.0551, -0.0281,
                                        -0.0599, -0.0498,  0.1913, -0.0012,  0.1343, -0.0535,  0.1148,  0.0752,
                                        -0.0608, -0.0023, -0.0280,  0.0357,  0.0447, -0.0258,  0.0825, -0.2215,
                                        -0.0424, -0.0056, -0.0749, -0.0908,  0.0577, -0.0546, -0.0545,  0.1290};


static const float phi_a_b1[PHI_A_V]  = {-0.0033,  0.0984, -0.1976,  0.1254};


static float out0[OUTPUT0];
static float phi_a[PHI_A_V];

static void  feedForwardMLP(float*);
static void  feedForwardPhiA0(float*, int);
static void  feedForwardPhiA1(float*, int);

void calcMlpOutput(float* input, float* out)
{

    feedForwardMLP(input);
    for(int k = 0; k < 4; k++)
    {
        out[k] = phi_a[k];    
    }
}

static void feedForwardMLP(float* inp)
{

    for(int k = 0; k < 32; k++)
    {
        feedForwardPhiA0(inp, k);
    }

    for(int k = 0; k < 4; k++)
    {
        feedForwardPhiA1(out0, k);
    }
    
}

static void feedForwardPhiA0(float* inp, int raw)
{
    float tmp       = 0;

    for(int k = 0; k < 24; k++)
    {
      tmp += inp[k]*phi_a_w0[raw][k];
    }

    out0[raw]    = (tmp + phi_a_b0[raw] >= 0) ? tmp + phi_a_b0[raw] : 0;

}

static void feedForwardPhiA1(float* inp, int raw)
{
  float tmp      = 0;
  for(int k = 0; k < 24; k++)
  {
    tmp += inp[k]*phi_a_w1[raw][k];
  }
  phi_a[raw]   = tmp + phi_a_b1[raw];
}





