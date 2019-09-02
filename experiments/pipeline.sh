#!/bin/bash -x
TJOB=(NONE)
EXPERIMENT=$1
NGENERATIONS=$2
NDJOBS=$3
mkdir $EXPERIMENT
mkdir $EXPERIMENT/models
ln -sfn $EXPERIMENT/models models

for (( GEN = 1; GEN <= $NGENERATIONS; GEN++ ))      ### Outer for loop ###
do
  DATADIR=$EXPERIMENT/data$GEN
  mkdir $DATADIR
  echo $TJOB

  DJOBS=()
<<<<<<< Updated upstream
  for (( djob = 1 ; djob <= $NDJOBS-1; djob++ )) ### Inner for loop ###
=======
  for (( djob = 1 ; djob < $NDJOBS; djob++ )) ### Inner for loop ###
>>>>>>> Stashed changes
  do
      DJOBS+=$(./runner.sh $DATADIR $TJOB)
      DJOBS+=:
  done
  DJOBS+=$(./runner.sh $DATADIR $TJOB) # one last one
  
  TJOB=$(sbatch --dependency=afterok:$DJOBS --parsable train_batcher.sh $GEN $EXPERIMENT)
done
