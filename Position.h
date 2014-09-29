#ifndef _POSITION__
#define _POSITION__

#ifdef __cplusplus
extern "C" {
#endif

struct 
{
	float x;	// [mm]
	float y;	// [mm]
	float phi;	// [rad], az x tengelyhez képest
} typedef Position;

#ifdef __cplusplus
}
#endif

#endif /* _POSITION__ */		