#!/usr/bin/python
# Stanley Lio, May 2014
import utm

def dms2dd(dms):
    d = dms[0]
    m = dms[1]
    s = dms[2]
    if m > 60 or s > 3600:
        print 'that''s unusual...'
    return d + m/60. + s/3600.

def dd2dms(deg):
    sign = int(deg/abs(deg))
    deg = abs(deg)
    import math
    d = int(math.floor(deg))
    m = int(math.floor((deg - d)*60.))
    s = int(round(((deg - d - m/60.0)*3600.)))
    return (sign,d,m,s)


if __name__ == '__main__':
    # test cases
    #print dd2dms(dms2dd((10,20,31)))
    #print dms2dd(dd2dms(25.7966))
    
    w = utm.from_latlon(38.37371667,-110.70876667)
    print 'In WGS84: Easting={}, Northing={}, Zone={}'.format(w[0],w[1],w[2])
    l = utm.to_latlon(w[0],w[1],w[2],'S')
    print 'In lat-long: Lat={}, Long={}'.format(l[0],l[1])
    latdms = dd2dms(l[0])
    londms = dd2dms(l[1])
    print 'or if you prefer dms, Lat={}d{}m{}s; Long={}d{}m{}s'.format(
        latdms[0]*latdms[1],latdms[2],latdms[3],londms[0]*londms[1],londms[2],londms[3])

