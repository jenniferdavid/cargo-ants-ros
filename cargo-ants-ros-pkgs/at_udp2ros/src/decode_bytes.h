#ifndef decode_bytes_H
#define decode_bytes_H

//std C++
#include <iostream>

union long2double {
	double d;
	unsigned long l;
};


inline short getInt16(unsigned char *buf, size_t start, bool BigEndian){
//Extracts single precission data from message
	short Z;
	if(BigEndian) 
	    //Big endian order
		Z =  buf[start] << 8 |  buf[start + 1];
    else
        //Little endian order
		Z =  buf[start+1] << 8 |  buf[start]; 
	
	return Z;
}


inline unsigned short getUint16(unsigned char *buf, size_t start, bool BigEndian){
//Extracts single precission data from message
	unsigned short Z;
	if(BigEndian) 
	    //Big endian order
		Z =  buf[start] << 8 |  buf[start + 1];
    else
        //Little endian order
		Z =  buf[start+1] << 8 |  buf[start]; 
	
	return Z;
}

inline int getInt32(unsigned char *buf, size_t start, bool BigEndian){
//Extracts single precission data from message
	int Z;
	if(BigEndian) 
	    //Big endian order
		Z =  buf[start] << 24 | buf[start+1] << 16 | buf[start+2] << 8 |  buf[start + 3];
    else
        //Little endian order
		Z = buf[start+3] << 24 | buf[start+2] << 16 | buf[start+1] << 8 |  buf[start]; 
	
	return Z;
}

inline unsigned int getUint32(unsigned char *buf, size_t start, bool BigEndian){
//Extracts single precission data from message
	unsigned int Z;
	if(BigEndian) 
	    //Big endian order
		Z = buf[start] << 24 | buf[start+1] << 16 | buf[start+2] << 8 |  buf[start + 3];
    else
        //Little endian order
		Z = buf[start+3] << 24 | buf[start+2] << 16 | buf[start+1] << 8 |  buf[start]; 
	
	return Z;
}

inline float getSingle(unsigned char *buf, size_t start, bool BigEndian){
//Extracts single precission data from message
	float singleData;
	int Z;
	if(BigEndian) 
	    //Big endian order
		Z = buf[start] << 24 | buf[start+1] << 16 | buf[start+2] << 8 |  buf[start + 3];
    else
        //Little endian order
		Z = buf[start+3] << 24 | buf[start+2] << 16 | buf[start+1] << 8 |  buf[start]; 
	
	singleData = *((float*)&Z);
		   
    return singleData;
	/* 
	  unsigned char a,b,c,d; // 8bits each
	  a=0x0a;b = 0x0b; c = 0x0c; d = 0x0d;
	  int z; //32bits
	  z = a << 24 | b << 16 | c << 8 |  d;  
	 * */    
}

inline double getDouble(unsigned char *buf, size_t start, bool BigEndian){
//Extracts single precission data from message
	
	long2double Z;
	//unsigned long ul; 
	
	if(BigEndian) 
	    //Big endian order
		Z.l = (unsigned long)buf[start] << 56 | (unsigned long)buf[start+1] << 48 | (unsigned long)buf[start+2] << 40 |  (unsigned long)buf[start + 3] << 32
			| (unsigned long)buf[start + 4] << 24 | (unsigned long)buf[start+5] << 16 | (unsigned long)buf[start+6] << 8 | (unsigned long)buf[start + 7];
    else
        //Little endian order
		Z.l= (unsigned long)buf[start+7] << 56 | (unsigned long)buf[start+6] << 48 | (unsigned long)buf[start+5] << 40 |  (unsigned long)buf[start + 4] << 32 
			| (unsigned long)buf[start + 3] << 24 | (unsigned long)buf[start+2] << 16 | (unsigned long)buf[start+1] << 8 |  (unsigned long)buf[start];
	   
    return Z.d;
}

#endif
