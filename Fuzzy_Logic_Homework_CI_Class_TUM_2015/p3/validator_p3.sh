#!/bin/bash

# Computational Intelligence Class

# Validator for Fuzzy Inference Systems
# Fuzzy logic controller for trajectory tracking
# 	input 3 values (x_ref, y_ref, theta_ref)
#	output 3 values (x_meas, y_meas, theta_meas)

# ******************************************************************************************************
# global variables
globalError=0;
toleratedGlobalError=20; # percentage of offset on the overall trajectory
toleratedPointError=5;   # cm / degrees
noutputs=3;
# ******************************************************************************************************
# function that reads two files and checks the output
validate_output() {

    # file descriptors associated with the 2 files
    local FD1=7;
    local FD2=8;

    # open the files specified as input
    exec 7<$1;
    exec 8<$2;

    # EOF flag for each of the files
    local eof1=0;
    local eof2=0;

    # the variables containing data on each line
    local data1;
    local data2;

    # remember a return value
    local errorVal=0;

    echo "      user output                    desired output                  difference                       STATUS"

    # while we haven't reached EOF of the files
    while ([ $eof1 -eq 0 ] || [ $eof2 -eq 0 ]);
    do

        # reinit values to 0
	data1=0;
	data2=0;

        # read line by line from each files
        if ! read data1 <&$FD1; then
            eof1=1
        fi
        if ! read data2 <&$FD2; then
            eof2=1
        fi

	# check if neither or both files are at EOF
	if (! [ $eof1 -eq $eof2 ] ); then 
		errorVal=2;
	fi

	# if reached EOF just return from call without computing anything
	if ([ $eof1 -eq 1 ] && [ $eof2 -eq 1 ]); then 
	    return $errorVal;
	fi
	
	# loop for each column in the output file
	for colidx in `seq 1 $noutputs`; do	

        # format our two numbers
	    cmd='{ printf( "%6.6f\n", $1) }';
	    data1a=`echo $data1 | cut -f$colidx -d',' | awk "$cmd"`;
	    data2a=`echo $data2 | cut -f$colidx -d',' | awk "$cmd"`;

		# check if we have a valid number
		if [ "`echo "$data1a" | grep "nan"`" != "" ]; then
            		echo "$data1a  $data2a  --- program output is not a number"
            		errorVal=3;
		else
			# compute the difference between two values
			cmd='{ printf( "%6.6f\n", $1-$2) }';
			data1a=${data1a#-};
			data2a=${data2a#-};
			diffA=`echo $data1a $data2a| awk "$cmd"`;
			diffA=${diffA#-};	

            		if [ $(echo "$diffA <= $toleratedPointError" | bc) -eq 1 ]; then
                		echo "      $data1a                        $data2a                        $diffA                        OK";
            		else
               			echo "      $data1a                        $data2a                        $diffA                        XXX";
				globalError=$((globalError+1));
                		errorVal=4;
            		fi
		fi
	done # end loop for each column
    done
}

# ******************************************************************************************************
echo ""

# determine length of files
progOutCount="`wc -l "$2" | cut -f1 -d' '`";
sampleCount="`wc -l "$3" | cut -f1 -d' '`";

if [ "${progOutCount}" -ne "${sampleCount}" ]
then
  echo "different number of output lines!";
  echo "";
  echo ${progOutCount};
  echo ${sampleCount};
  exit 2;
fi;

echo "";
echo "(constant) toleratedPointError:  $toleratedPointError";
echo "";

# compute the squared errors
validate_output $2 $3
errorVal=$?;			# remember error indicator

# check global errror after reading the input files
cmd='{ printf( "%6.6f\n", $1*100/$2) }';
globalError=`echo $globalError $sampleCount | awk "$cmd"`;
if [ $(echo "$globalError <= $toleratedGlobalError" | bc) -eq 1 ]; then
    echo "totalError:      $globalError 	OK"
else
    echo "totalError:      $globalError 	XXX"
    errorVal=5;
fi

exit $errorVal;
