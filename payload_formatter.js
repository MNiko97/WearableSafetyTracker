function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};
    
    // HeartBeat
    rawHb = bytes[0];
    decoded.hb = rawHb;
    
    // button 
    rawButton = bytes[1];
    decoded.button = rawButton ;
    
    // latitude
    rawLat1 = bytes[2];
    rawLat2 = bytes[3] + bytes[4] * 256;
    decoded.lat = rawLat1 + rawLat2/10000;
    
    // longitude
    rawLon1 = bytes[5];
    rawLon2 = bytes[6] + bytes[7] * 256;
    decoded.lon = rawLon1 + rawLon2/10000;
  
    // speed
    rawSpeed = bytes[8] + bytes[9] * 256;
    decoded.speed = sflt162f(rawSpeed) * 18.5;
    
    // altitude
    rawAlt = bytes[10] + bytes[11] * 256;
    decoded.alt = sflt162f(rawAlt) * 10000 ;
    
    // sat co
    rawSatco = bytes[12] ;
    decoded.satco = rawSatco;
    
    // sat co qual
    rawSigqual = bytes[13];
    decoded.sigqual = rawSigqual;
    
    // falling
    rawFall = bytes[14];
    decoded.fall = rawFall;
    
    return decoded;
}

function sflt162f(rawSflt16) {
  // throw away high bits for repeatability.
  rawSflt16 &= 0xFFFF;

  // special case minus zero:
  if (rawSflt16 == 0x8000)
      return -0.0;

  // extract the sign.
  var sSign = ((rawSflt16 & 0x8000) != 0) ? -1 : 1;

  // extract the exponent
  var exp1 = (rawSflt16 >> 11) & 0xF;

  // extract the "mantissa" (the fractional part)
  var mant1 = (rawSflt16 & 0x7FF) / 2048.0;

  // convert back to a floating point number. We hope 
  // that Math.pow(2, k) is handled efficiently by
  // the JS interpreter! If this is time critical code,
  // you can replace by a suitable shift and divide.
  var f_unscaled = sSign * mant1 * Math.pow(2, exp1 - 15);

  return f_unscaled;
}
  
