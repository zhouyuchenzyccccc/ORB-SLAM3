/**
* This file is part of ORB-SLAM3
*/

#include <algorithm>
#include <chrono>
#include <cctype>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

struct FrameRGBD
{
    string frame_index;
    string rgb_path;
    string depth_path;
    double ts;
};

static inline string Trim(const string &s)
{
    size_t b = 0;
    while (b < s.size() && isspace(static_cast<unsigned char>(s[b])))
        ++b;
    size_t e = s.size();
    while (e > b && isspace(static_cast<unsigned char>(s[e - 1])))
        --e;
    return s.substr(b, e - b);
}

static bool EndsWith(const string &s, const string &suffix)
{
    if (suffix.size() > s.size())
        return false;
    return equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

static string JoinPath(const string &a, const string &b)
{
    if (a.empty())
        return b;
    if (a.back() == '/')
        return a + b;
    return a + "/" + b;
}

static string Stem(const string &name)
{
    const size_t pos = name.find_last_of('.');
    if (pos == string::npos)
        return name;
    return name.substr(0, pos);
}

static bool IsAllowedExt(const string &name, const vector<string> &exts)
{
    for (size_t i = 0; i < exts.size(); ++i)
    {
        if (EndsWith(name, exts[i]))
            return true;
    }
    return false;
}

static bool LoadImageMap(const string &folder, const vector<string> &exts, map<string, string> &out)
{
    DIR *dir = opendir(folder.c_str());
    if (!dir)
        return false;

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL)
    {
        const string name = ent->d_name;
        if (name == "." || name == "..")
            continue;
        if (!IsAllowedExt(name, exts))
            continue;

        out[Stem(name)] = JoinPath(folder, name);
    }

    closedir(dir);
    return !out.empty();
}

static bool LoadRefTsFromImuCsv(const string &imu_csv_path, map<string, double> &ts_map)
{
    ifstream fin(imu_csv_path.c_str());
    if (!fin.is_open())
        return false;

    string line;
    bool header_read = false;
    while (getline(fin, line))
    {
        line = Trim(line);
        if (line.empty())
            continue;

        if (!header_read)
        {
            header_read = true;
            if (line.find("frame_index") != string::npos)
                continue;
        }

        string token;
        stringstream ss(line);
        vector<string> cols;
        while (getline(ss, token, ','))
            cols.push_back(Trim(token));

        if (cols.size() < 2)
            continue;

        ts_map[cols[0]] = stod(cols[1]) * 1e-6;
    }

    return !ts_map.empty();
}

int main(int argc, char **argv)
{
    if (argc < 4 || argc > 6)
    {
        cerr << endl
             << "Usage: ./rgbd_local_dataset path_to_vocabulary path_to_settings path_to_sequence_folder (trajectory_file_name) (fps_if_no_imu_csv)" << endl;
        cerr << "Expected sequence layout: <seq>/RGB, <seq>/Depth, optional <seq>/IMU/imu.csv" << endl;
        return 1;
    }

    const string seq_path = string(argv[3]);
    const string rgb_folder = JoinPath(seq_path, "RGB");
    const string depth_folder = JoinPath(seq_path, "Depth");
    const string imu_csv_path = JoinPath(JoinPath(seq_path, "IMU"), "imu.csv");

    string traj_name = "CameraTrajectory.txt";
    string kf_name = "KeyFrameTrajectory.txt";
    if (argc >= 5)
    {
        traj_name = string(argv[4]);
        kf_name = string("kf_") + traj_name;
    }

    double fps = 30.0;
    if (argc == 6)
        fps = stod(argv[5]);
    if (fps <= 0.0)
        fps = 30.0;

    map<string, string> rgb_map;
    map<string, string> depth_map;
    if (!LoadImageMap(rgb_folder, {".jpg", ".jpeg", ".png", ".bmp"}, rgb_map))
    {
        cerr << "Failed to load RGB images from " << rgb_folder << endl;
        return 1;
    }
    if (!LoadImageMap(depth_folder, {".png", ".tiff", ".tif", ".exr"}, depth_map))
    {
        cerr << "Failed to load depth images from " << depth_folder << endl;
        return 1;
    }

    map<string, double> ts_map;
    const bool has_imu_ts = LoadRefTsFromImuCsv(imu_csv_path, ts_map);

    vector<FrameRGBD> frames;
    frames.reserve(min(rgb_map.size(), depth_map.size()));

    size_t idx = 0;
    for (map<string, string>::const_iterator it = rgb_map.begin(); it != rgb_map.end(); ++it)
    {
        map<string, string>::const_iterator it_d = depth_map.find(it->first);
        if (it_d == depth_map.end())
            continue;

        FrameRGBD fr;
        fr.frame_index = it->first;
        fr.rgb_path = it->second;
        fr.depth_path = it_d->second;

        map<string, double>::const_iterator it_ts = ts_map.find(fr.frame_index);
        if (has_imu_ts && it_ts != ts_map.end())
            fr.ts = it_ts->second;
        else
            fr.ts = static_cast<double>(idx) / fps;

        frames.push_back(fr);
        ++idx;
    }

    if (frames.empty())
    {
        cerr << "No matched RGB-Depth pairs found." << endl;
        return 1;
    }

    sort(frames.begin(), frames.end(), [](const FrameRGBD &a, const FrameRGBD &b)
         { return a.ts < b.ts; });

    cout << "Frames loaded: " << frames.size() << endl;
    if (has_imu_ts)
        cout << "Using timestamps from " << imu_csv_path << endl;
    else
        cout << "imu.csv not found, using synthetic timestamps by FPS=" << fps << endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    float imageScale = SLAM.GetImageScale();

    vector<float> vTimesTrack;
    vTimesTrack.resize(frames.size());

    for (size_t i = 0; i < frames.size(); ++i)
    {
        const FrameRGBD &fr = frames[i];
        cv::Mat imRGB = cv::imread(fr.rgb_path, cv::IMREAD_UNCHANGED);
        cv::Mat imD = cv::imread(fr.depth_path, cv::IMREAD_UNCHANGED);
        if (imRGB.empty() || imD.empty())
        {
            cerr << "Failed to read frame " << fr.frame_index << endl;
            return 1;
        }

        if (imageScale != 1.f)
        {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        SLAM.TrackRGBD(imRGB, imD, fr.ts, fr.frame_index);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        const double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[i] = ttrack;

        double T = 0.0;
        if (i + 1 < frames.size())
            T = frames[i + 1].ts - fr.ts;
        else if (i > 0)
            T = fr.ts - frames[i - 1].ts;
        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    SLAM.Shutdown();

    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0.0f;
    for (size_t i = 0; i < vTimesTrack.size(); ++i)
        totaltime += vTimesTrack[i];

    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] << endl;
    cout << "mean tracking time: " << totaltime / vTimesTrack.size() << endl;

    SLAM.SaveTrajectoryTUM(traj_name);
    SLAM.SaveKeyFrameTrajectoryTUM(kf_name);
    return 0;
}
