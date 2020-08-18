#!/bin/bash
#Simple array
jetheight=(22.5 27.5 35)
#jetheight=(15 20 25)
#noise_level=(0.0 0.025 0.05 0.1 0.15 0.2)
#noise_offset=(0.0 0.1 0.25 0.5 0.75 1)
#alpha_twist=(0.0 0.2 0.3 0.7 1.2)
#betha_twist=(0.0 0.01 0.05 0.075 0.1 0.15)
#gamma_twist=(0.0 0.01 0.05 0.075 0.1 0.15)
#x_off=(0.0 10. 50. 75. 100.)
#y_off=(0.0 10. 20. 30. 40. 50. 75. 100.)
#jet_setting=(0 1 2)
#min_scale_SIFT=(0.1 0.25 0.5 0.7 1.3)
#nr_octaves_SIFT=(3 4 5 6 7)
#nr_scales_per_octave_SIFT=(4 5 6 7 8)
#min_contrast_SIFT=(0.1 0.25 0.5 0.7 .3)
SalientRad_muliplier_ISS=(5 6 7)
NonMaxMultiplier_ISS=(3 4 5)
Threshold21_ISS=(0.8 0.99 1.2)
Threshold32_ISS=(0.8 0.99 1.2)
setMinNeighbors_ISS=(4 5 6)
setNumberOfThreads_ISS=(1 2)


for SalientRad_muliplier_ISS_counter in "${SalientRad_muliplier_ISS[@]}"
do
  for NonMaxMultiplier_ISS_counter in "${NonMaxMultiplier_ISS[@]}"
  do
    for Threshold21_ISS_counter in "${Threshold21_ISS[@]}"
    do
      for Threshold32_ISS_counter in "${Threshold32_ISS[@]}"
      do
        for setMinNeighbors_ISS_counter in "${setMinNeighbors_ISS[@]}"
        do
          for setNumberOfThreads_ISS_counter in "${setNumberOfThreads_ISS[@]}"
          do
            for repeat_counter in "${repeat[@]}"
              do
                echo $repeat_counter
                 #echo $min_contrast_SIFT_counter
                 #./test -o $noise_off_counter -n $noise_counter -a $alpha_twist_counter -b 0.0 -c 0.0 -x $xy_off_counter -y $xy_off_counter -z 0 -j 2 -e 0.1 -v 0.2 -d 50. -q 1 -i $jet_counter -f $varname
                 #./testkeypoints --noiseoff=$noise_off_counter --noisestdv=$noise_counter --method=SWIFT --grid=1 -j $jet_set_counter --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_2.pcd --output_filename=../measurements/SIFTtuning.csv  --min_scale_SIFT=$min_scale_SIFT_counter --nr_octaves_SIFT=$nr_octaves_SIFT_counter --nr_scales_per_octave_SIFT=$nr_scales_per_octave_SIFT_counter --min_contrast_SIFT=$min_contrast_SIFT_counter
                 #./testkeypoints --noiseoff=$noise_off_counter --noisestdv=0.0 --method=SWIFT --visdistance=30 --grid=1 -j 0 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_2.pcd --grid=1 --output_filename=deletelater.csv --min_scale_SIFT=0.1 --nr_octaves_SIFT=3 --nr_scales_per_octave_SIFT=6 --min_contrast_SIFT=0.05
                 #./testkeypoints --noiseoff=$noise_off_counter --noisestdv=0.0 --method=SWIFT --visdistance=30 -j 0 --grid=1 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_2.pcd -g 1 --output_filename=deletelater.csv --min_scale_SIFT=0.1 --nr_octaves_SIFT=3 --nr_scales_per_octave_SIFT=6 --min_contrast_SIFT=0.05
                ./testkeypoints --noiseoff=0.0 --noisestdv=0.1 --method=ISS --grid=1 -j 0 --input_filename_1=../../data/cloud_overlap_1.pcd --input_filename_2=../../data/cloud_overlap_1.pcd --output_filename=../measurements/ISStuning_1_map1.csv  --SalientRad_muliplier_ISS=$SalientRad_muliplier_ISS_counter --NonMaxMultiplier_ISS=$NonMaxMultiplier_ISS_counter --Threshold21_ISS=$Threshold21_ISS_counter --Threshold32_ISS=$Threshold32_ISS_counter --setMinNeighbors_ISS=$setMinNeighbors_ISS_counter --setNumberOfThreads_ISS=$setNumberOfThreads_ISS_counter
                ./testkeypoints --noiseoff=0.0 --noisestdv=0.1 --method=ISS --grid=1 -j 0 --input_filename_1=../../data/cloud_nocoor.pcd --input_filename_2=../../data/cloud_overlap__big_2.pcd --output_filename=../measurements/ISStuning_1_map2.csv  --SalientRad_muliplier_ISS=$SalientRad_muliplier_ISS_counter --NonMaxMultiplier_ISS=$NonMaxMultiplier_ISS_counter --Threshold21_ISS=$Threshold21_ISS_counter --Threshold32_ISS=$Threshold32_ISS_counter --setMinNeighbors_ISS=$setMinNeighbors_ISS_counter --setNumberOfThreads_ISS=$setNumberOfThreads_ISS_counter
            done
          done
        done
      done
    done
  done
done




