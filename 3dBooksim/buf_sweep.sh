#! /bin/sh

cur_time=`date +%d_%m_%Y_%H_%M`
new_dir=results/${cur_time}
mkdir $new_dir
echo "Created new directory : ${new_dir}"
echo "Press enter to proceed."
read

echo "Enter one line description of the simulation being run:"
echo ""
read description
echo $description >> ${new_dir}/README
echo "" >> ${new_dir}/README
# copy the overridden parameters for future reference
echo "$* results_dir=${new_dir}" >> ${new_dir}/README
# copy the config file for future reference
cp examples/arpit/1d_ring ${new_dir}

for traffic in uniform tornado transpose bitcomp
do
	for buf_size in 2 4 8 16 32 64
	do
		../sweep.sh ./booksim examples/pras/3dmesh traffic=${traffic} vc_buf_size=${buf_size} $* results_dir=${new_dir}
	done
done

echo "Results directory => ${new_dir}"
