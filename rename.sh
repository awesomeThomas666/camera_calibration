#! /bin/bash
i=1
for var in *.jpg
do 
    mv -f $var $i.jpg
    let i=i+1
done
