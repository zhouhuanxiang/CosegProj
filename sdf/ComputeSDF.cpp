#include "Fuser.h"

extern void load_trajectory(const std::string& filename, std::vector<ml::mat4f>& trajectory);

void run_sdf(std::string& inputfile, std::string& sdf_file) {
	int t = inputfile.size() - 1;
	while (inputfile[t] != '\\')
		t -= 1;
	t += 1;
	std::vector<ml::mat4f> trajectory;
	std::string filename = inputfile.substr(t, inputfile.size() - t - 5);
	const std::string prefix = std::string("sens-corrs\\") + filename;
	const std::string corr_file = prefix + ".corrs";
	const std::string traj_file = prefix + ".traj";
	load_trajectory(traj_file, trajectory);

	Fuser fuser;
	fuser.fuse(sdf_file, inputfile, trajectory);
}