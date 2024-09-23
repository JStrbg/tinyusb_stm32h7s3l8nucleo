head -c 200K </dev/urandom > /run/media/nnms/0000-1234/test.dat
dd if=/run/media/nnms/0000-1234/test.dat of=/dev/null bs=512 count=200 iflag=direct