/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gomez Rodriguez, Jose M.M. Montiel and Juan D. Tardos, University of Zaragoza.
* Copyright (C) 2014-2016 Raul Mur-Artal, Jose M.M. Montiel and Juan D. Tardos, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

struct FrameRecord
{
    string frame_index;
    double ref_ts;
    double accel_ts;
    float ax;
    float ay;
    float az;
    double gyro_ts;
    float gx;
    float gy;
    float gz;
    string rgb_path;
    string depth_path;
};

struct ImuRawRecord
{
    double ts;
    bool is_gyro;
    float x;
    float y;
    float z;
};

static ORB_SLAM3::IMU::Point BuildImuPoint(float ax, float ay, float az, float gx, float gy, float gz, double ts)
{
    return ORB_SLAM3::IMU::Point(ax, ay, az, gx, gy, gz, ts);
}

static float LerpFloat(float a, float b, float r)
{
    return a + (b - a) * r;
}

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

static bool FileExists(const string &path)
{
    ifstream f(path.c_str());
    return f.good();
}

static vector<string> SplitCsv(const string &line)
{
    vector<string> out;
    string token;
    stringstream ss(line);
    while (getline(ss, token, ','))
    {
        out.push_back(Trim(token));
    }
    return out;
}

static string JoinPath(const string &a, const string &b)
{
    if (a.empty())
        return b;
    if (a.back() == '/')
        return a + b;
    return a + "/" + b;
}

static string FindImageWithKnownExt(const string &folder, const string &stem, const vector<string> &exts)
{
    for (size_t i = 0; i < exts.size(); ++i)
    {
        const string candidate = JoinPath(folder, stem + exts[i]);
        if (FileExists(candidate))
            return candidate;
    }
    return string();
}

static bool LoadFrameCsv(const string &imu_csv_path, const string &seq_path, vector<FrameRecord> &frames)
{
    ifstream fin(imu_csv_path.c_str());
    if (!fin.is_open())
        return false;

    const string rgb_folder = JoinPath(seq_path, "RGB");
    const string depth_folder = JoinPath(seq_path, "Depth");
    const vector<string> rgb_exts = {".jpg", ".jpeg", ".png", ".bmp"};
    const vector<string> depth_exts = {".png", ".tiff", ".tif", ".exr"};

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

        vector<string> cols = SplitCsv(line);
        if (cols.size() < 10)
            continue;

        FrameRecord rec;
        rec.frame_index = cols[0];
        rec.ref_ts = stod(cols[1]) * 1e-6;
        rec.accel_ts = stod(cols[2]) * 1e-6;
        rec.ax = stof(cols[3]);
        rec.ay = stof(cols[4]);
        rec.az = stof(cols[5]);
        rec.gyro_ts = stod(cols[6]) * 1e-6;
        rec.gx = stof(cols[7]);
        rec.gy = stof(cols[8]);
        rec.gz = stof(cols[9]);

        rec.rgb_path = FindImageWithKnownExt(rgb_folder, rec.frame_index, rgb_exts);
        rec.depth_path = FindImageWithKnownExt(depth_folder, rec.frame_index, depth_exts);
        if (rec.rgb_path.empty() || rec.depth_path.empty())
            continue;

        frames.push_back(rec);
    }

    sort(frames.begin(), frames.end(), [](const FrameRecord &a, const FrameRecord &b)
         { return a.ref_ts < b.ref_ts; });
    return !frames.empty();
}

static bool LoadImuRawCsv(const string &imu_raw_csv_path, vector<ImuRawRecord> &accel, vector<ImuRawRecord> &gyro, double imu_time_shift_sec)
{
    ifstream fin(imu_raw_csv_path.c_str());
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
            if (line.find("ts_us") != string::npos)
                continue;
        }

        vector<string> cols = SplitCsv(line);
        if (cols.size() < 5)
            continue;

        ImuRawRecord rec;
        rec.ts = stod(cols[0]) * 1e-6 + imu_time_shift_sec;
        rec.is_gyro = (cols[1] == "gyro");
        rec.x = stof(cols[2]);
        rec.y = stof(cols[3]);
        rec.z = stof(cols[4]);

        if (rec.is_gyro)
            gyro.push_back(rec);
        else
            accel.push_back(rec);
    }

    sort(accel.begin(), accel.end(), [](const ImuRawRecord &a, const ImuRawRecord &b)
         { return a.ts < b.ts; });
    sort(gyro.begin(), gyro.end(), [](const ImuRawRecord &a, const ImuRawRecord &b)
         { return a.ts < b.ts; });

    return !accel.empty() && !gyro.empty();
}

int main(int argc, char **argv)
{
    if (argc < 4 || argc > 5)
    {
        cerr << endl
             << "Usage: ./rgbd_inertial_local_dataset path_to_vocabulary path_to_settings path_to_sequence_folder (trajectory_file_name)" << endl;
        cerr << "Expected sequence layout: <seq>/RGB, <seq>/Depth, <seq>/IMU/imu.csv, <seq>/IMU/imu_raw.csv" << endl;
        return 1;
    }

    const string seq_path = string(argv[3]);
    const string imu_csv_path = JoinPath(JoinPath(seq_path, "IMU"), "imu.csv");
    const string imu_raw_csv_path = JoinPath(JoinPath(seq_path, "IMU"), "imu_raw.csv");

    vector<FrameRecord> frames;
    if (!LoadFrameCsv(imu_csv_path, seq_path, frames))
    {
        cerr << "Failed to load valid frame data from " << imu_csv_path << endl;
        return 1;
    }

    const char *use_raw_env = getenv("ORB3_USE_IMU_RAW");
    const bool use_raw = (use_raw_env && string(use_raw_env) == "1");

    double imu_time_shift_sec = 0.0;
    const char *imu_shift_env = getenv("ORB3_IMU_TIMESHIFT");
    if (imu_shift_env)
    {
        try
        {
            imu_time_shift_sec = stod(string(imu_shift_env));
        }
        catch (...)
        {
            cerr << "Warning: invalid ORB3_IMU_TIMESHIFT='" << imu_shift_env << "', fallback to 0" << endl;
            imu_time_shift_sec = 0.0;
        }
    }

    vector<ImuRawRecord> accel_raw;
    vector<ImuRawRecord> gyro_raw;
    const bool has_raw = use_raw && LoadImuRawCsv(imu_raw_csv_path, accel_raw, gyro_raw, imu_time_shift_sec);

    cout << "Frames loaded: " << frames.size() << endl;
    if (has_raw)
        cout << "Loaded imu_raw.csv with " << accel_raw.size() << " accel and " << gyro_raw.size() << " gyro samples" << endl;
    else if (use_raw)
        cout << "imu_raw.csv unavailable or invalid, fallback to frame-synced imu.csv only" << endl;
    else
        cout << "Using frame-synced imu.csv only (set ORB3_USE_IMU_RAW=1 to enable imu_raw densification)" << endl;
    cout << "IMU raw timestamp shift: " << imu_time_shift_sec << " s" << endl;

    string traj_name = "CameraTrajectory.txt";
    string kf_name = "KeyFrameTrajectory.txt";
    if (argc == 5)
    {
        traj_name = string(argv[4]);
        kf_name = string("kf_") + traj_name;
    }

    const char *no_viewer_env = getenv("ORB3_NO_VIEWER");
    const bool use_viewer = !(no_viewer_env && string(no_viewer_env) == "1");
    cout << "Viewer: " << (use_viewer ? "ON" : "OFF") << endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, use_viewer);
    float imageScale = SLAM.GetImageScale();

    vector<float> vTimesTrack;
    vTimesTrack.resize(frames.size());

    size_t accel_idx = 0;
    size_t gyro_idx = 0;

    for (size_t i = 0; i < frames.size(); ++i)
    {
        const FrameRecord &fr = frames[i];
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

        vector<ORB_SLAM3::IMU::Point> vImuMeas;

        // Primary policy: imu.csv is frame-synchronized; densify with interpolation.
        if (i > 0)
        {
            const FrameRecord &prev = frames[i - 1];
            const double t_begin = prev.ref_ts + 1e-6;
            const double t_end = fr.ref_ts - 1e-6;

            const int kInterpSamples = 8;
            for (int s = 0; s < kInterpSamples; ++s)
            {
                const float r = static_cast<float>(s) / static_cast<float>(kInterpSamples - 1);
                double ts = t_begin + (t_end - t_begin) * r;
                if (s == 0)
                    ts = t_begin;
                else if (s == kInterpSamples - 1)
                    ts = t_end;

                vImuMeas.push_back(BuildImuPoint(
                    LerpFloat(prev.ax, fr.ax, r),
                    LerpFloat(prev.ay, fr.ay, r),
                    LerpFloat(prev.az, fr.az, r),
                    LerpFloat(prev.gx, fr.gx, r),
                    LerpFloat(prev.gy, fr.gy, r),
                    LerpFloat(prev.gz, fr.gz, r),
                    ts));
            }
        }
        else
        {
            const double t_begin = fr.ref_ts - 0.005;
            const double t_end = fr.ref_ts - 1e-6;
            vImuMeas.push_back(BuildImuPoint(fr.ax, fr.ay, fr.az, fr.gx, fr.gy, fr.gz, t_begin));
            vImuMeas.push_back(BuildImuPoint(fr.ax, fr.ay, fr.az, fr.gx, fr.gy, fr.gz, t_end));
        }

        // Optional densification from imu_raw.csv within (prev_frame_ts, current_frame_ts].
        if (has_raw && i > 0)
        {
            const double t_begin = frames[i - 1].ref_ts;
            const double t_end = fr.ref_ts;

            while (accel_idx + 1 < accel_raw.size() && accel_raw[accel_idx + 1].ts <= t_begin)
                ++accel_idx;
            while (gyro_idx < gyro_raw.size() && gyro_raw[gyro_idx].ts <= t_begin)
                ++gyro_idx;

            vector<ORB_SLAM3::IMU::Point> vDense;
            while (gyro_idx < gyro_raw.size() && gyro_raw[gyro_idx].ts <= t_end)
            {
                const ImuRawRecord &g = gyro_raw[gyro_idx];
                while (accel_idx + 1 < accel_raw.size() && accel_raw[accel_idx + 1].ts <= g.ts)
                    ++accel_idx;

                const ImuRawRecord &a = accel_raw[accel_idx];
                double tg = g.ts;
                if (tg < t_begin + 1e-6)
                    tg = t_begin + 1e-6;
                if (tg > t_end - 1e-6)
                    tg = t_end - 1e-6;

                vDense.push_back(BuildImuPoint(a.x, a.y, a.z, g.x, g.y, g.z, tg));
                ++gyro_idx;
            }

            if (vDense.size() >= 2)
                vImuMeas.swap(vDense);
        }

        sort(vImuMeas.begin(), vImuMeas.end(), [](const ORB_SLAM3::IMU::Point &a, const ORB_SLAM3::IMU::Point &b)
             { return a.t < b.t; });
        for (size_t k = 1; k < vImuMeas.size(); ++k)
        {
            if (vImuMeas[k].t <= vImuMeas[k - 1].t)
                vImuMeas[k].t = vImuMeas[k - 1].t + 1e-6;
        }

        if (i > 0)
        {
            const double tmin = frames[i - 1].ref_ts + 1e-6;
            const double tmax = fr.ref_ts - 1e-6;
            if (vImuMeas.front().t < tmin)
                vImuMeas.front().t = tmin;
            if (vImuMeas.back().t > tmax)
                vImuMeas.back().t = tmax;
            if (vImuMeas.back().t <= vImuMeas.front().t)
                vImuMeas.back().t = vImuMeas.front().t + 1e-4;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        SLAM.TrackRGBD(imRGB, imD, fr.ref_ts, vImuMeas, fr.frame_index);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        const double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[i] = ttrack;

        double T = 0.0;
        if (i + 1 < frames.size())
            T = frames[i + 1].ref_ts - fr.ref_ts;
        else if (i > 0)
            T = fr.ref_ts - frames[i - 1].ref_ts;
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
