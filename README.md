# Naza GPS Decoder

Naza gps serial decoder, taken from
- http://www.rcgroups.com/forums/showthread.php?t=1995704a
- https://github.com/mandersonian/minimosd-naza-frsky/blob/master/NazaDecoderLib.cppa
- https://developer.mbed.org/users/garfield38/code/NazaDecoder/rev/b0ba4e08a18c

Usage:

        decoder = NazaGpsDecoder()
        a = decoder.readRawMessage()
        print "SELFTEST",a
        print "SELFTEST",decoder.decodeMessage(a)
        print "SELFTEST",decoder.readMessage("ver")
        print "SELFTEST",decoder.readMessage("com")
        print "SELFTEST",decoder.readMessage("gps")

