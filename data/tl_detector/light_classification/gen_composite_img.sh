#!/bin/bash
#
# Utility script to generate annotated data for udacity simulator traffic light det model.
#
# Description:
# Randomly chooses a traffic light patch image and superimposes at a randomly chosen location in a random background image.
# The patch is chosen from either $img_path/Red/, $img_path/Green/, $img_path/Yellow/
# The background is chosen from $img_path/Background/
# The resulting image is written to $img_path/Composite/, and associated annotation data written to stdout.
#
# Inputs:
# - img_path: filesystem path to location of image patches and backgrounds 
#
# Output:
# - writes the composite image to $img_path/composite/
# - outputs an annotation line to stdout
#
# Example usage:
# ~$ ./gen_composite_img.sh /path/to/images/ >> /path/to/groundtruth.csv
#
# Example usage:
# ~$ for i in `seq 0 100`; do ./gen_composite_img.sh . >> composite_gt.csv; done
#
img_path=$1

composite_prefix="Composite"
background_prefix="Background"

states=("Green" "Red" "Yellow")
let rand_state=$RANDOM%3
trafficlight_prefix=${states[$rand_state]}

let rand_bg=$RANDOM%3 # magic number from count of available background images on filesystem
let rand_tl=$RANDOM%3 # magic number from count of available patch images on filesystem

# assuming images named by just an index number and .png extension
background_img_path=$img_path"/"$background_prefix"/"$rand_bg".png"
trafficlight_img_path=$img_path"/"$trafficlight_prefix"/"$rand_tl".png"

# default background dimensions
bg_img_w=800
bg_img_h=600

# get image dimensions
bg_img_w=`identify -format '%w' $background_img_path`
bg_img_h=`identify -format '%h' $background_img_path`
tl_img_w=`identify -format '%w' $trafficlight_img_path`
tl_img_h=`identify -format '%h' $trafficlight_img_path`

# choose a random location within the background image to superimpose
let rand_x=$RANDOM%$bg_img_w
let rand_y=$RANDOM%$bg_img_h

# ensure within image bounds
pos_x=$(expr $rand_x - $tl_img_w)
pos_y=$(expr $rand_y - $tl_img_h)
marker_pos_x=$(( $pos_x > 0 ? $pos_x : 0))
marker_pos_y=$(( $pos_y > 0 ? $pos_y : 0))

# choose target location
composite_img_name=$rand_bg"-"$rand_tl"-"$marker_pos_x"-"$marker_pos_y".png"
composite_img_path=$img_path"/"$composite_prefix"/"$composite_img_name

# if image doesn't already exist, we continure
if [[ ! -f $composite_img_path ]]; then

	# performing overlay and writing to filesystem
	convert  $background_img_path $trafficlight_img_path -geometry +$marker_pos_x+$marker_pos_y -composite $composite_img_path

	# determining bottom right positions required for ground truth CSV file
	xmax=$(expr $marker_pos_x + $tl_img_w)
	ymax=$(expr $marker_pos_y + $tl_img_h)

	# writing the associated ground truth to stdout
	# output format: 'image_name', 'xmin', 'ymin', 'xmax', 'ymax', 'state'
	echo $composite_img_name","$marker_pos_x","$marker_pos_y","$xmax","$ymax","$rand_state

fi
