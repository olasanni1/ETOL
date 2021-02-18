/******************************************************************************/
/*!
 * @file
 *
 * @copyright 2020 Olatunde Sanni
 * @author Olatunde Sanni <olasanni1@gmail.com>
 * @date 20 April 2020
 * @version 1.0.1
 * @brief The implementation for the TrajectoryOptimizer class
 * @section DESCRIPTION
 * An abstract class for loading, saving, and visualizing trajectory
 * optimization problems and solutions.
 ******************************************************************************/

#include <CGAL/basic.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>
#include <libxml/xmlreader.h>
#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <gnuplot-iostream.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cassert>
#include <cfloat>
#include <fstream>
#include <iostream>
#include <xmlutils.hpp>
#include <ETOL/TrajectoryOptimizer.hpp>

inline void mysleep(unsigned millis) {
        ::usleep(millis * 1000);
}

#define MY_ENCODING "UTF-8"                                /* XML encoding */
#define MAXLENGTH 10                                    /* char array size */

namespace ETOL {

// Constructor

TrajectoryOptimizer::TrajectoryOptimizer() {
    this->_score = 0.;
    this->_nControls = 0;
    this->_nStates = 0;
    this->_dt = 0.0;
    this->_nSteps = 0;
    this->_xrhorizon = 0;
    this->_urhorizon = 0;
    this->_rhorizon = 0;
    this->_x0 = {};
    this->_xf = {};
    this->_xtol = {};
    this->_xlower = {};
    this->_xupper = {};
    this->_xvartype = {};
    this->_ulower = {};
    this->_uupper = {};
    this->_uvartype = {};
    this->_xtraj = traj_t();
    this->_utraj = traj_t();
    this->_maximize = false;
    this->_objective = NULL;
    this->_obstacles_raw = {};
    this->_obstacles = {};
    this->_tracks = {};
    this->_eAny = NULL;
}

// Static Functions

/**
 * Uses CGAL's monotone_partition_2 to partition a polygon into convex polygons.
 * Each polygon has a lower and upper segment.
 * Each segment is sorted from left to right.
 */
region_t TrajectoryOptimizer::genRegion(border_t* border) {
    region_t region;

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef CGAL::Partition_traits_2<K>                         Traits;
    typedef Traits::Polygon_2                                   Polygon_2;
    typedef std::list<Polygon_2>                                Polygon_list;

    Polygon_2                                                   polygon;
    Polygon_list                                                partition_polys;
    border_t::iterator                                          it;
    std::list<Polygon_2>::const_iterator                        poly_it;

    // Add corners to polygon object
    for (it = border->begin(); it != border->end(); it++)
        polygon.push_back(Traits::Point_2((*it).at(0), (*it).at(1)));

    // Compute polygon partitions
    CGAL::optimal_convex_partition_2(polygon.vertices_begin(),
                                    polygon.vertices_end(),
                                    std::back_inserter(partition_polys));

    // Find lower and upper segments
    for (poly_it = partition_polys.begin(); poly_it != partition_polys.end();
            poly_it++) {
        assert(CGAL::is_y_monotone_2((*poly_it).vertices_begin(),
                                       (*poly_it).vertices_end()));
        border_t seg1, seg2;
        border_t::iterator seg1_it, seg2_it;
        Polygon_2::Vertex_const_iterator v_it;
        Polygon_2::Vertex_circulator c, cleft, cright;

        // Find left and right vertices in circulator
        c = (*poly_it).vertices_circulator();
        if (c != NULL) {
            do {
                if (*c == (*(*poly_it).left_vertex()))
                    cleft = c;
                else if (*c == (*(*poly_it).right_vertex()))
                    cright = c;
            } while (++c != (*poly_it).vertices_circulator());
        }
        // Copy from left to right in + direction
        c = cleft;
        do {
            seg1.push_back({(*c).x(), (*c).y(), 0.});
        } while (++c != cright);
        seg1.push_back({(*c).x(), (*c).y(), 0.});
        // Copy from left to right in - direction
        c = cright;
        do {
            seg2.push_back({(*c).x(), (*c).y(), 0.});
        } while (++c !=cleft);
        seg2.push_back({(*c).x(), (*c).y(), 0.});
        seg2.reverse();

        // Find lower and upper segments
        v_it = (*poly_it).top_vertex();
        corner_t vtop = {(*v_it).x(), (*v_it).y(), 0.0};
        seg1_it = std::find(seg1.begin(), seg1.end(), vtop);
        seg2_it = std::find(seg2.begin(), seg2.end(), vtop);
        if (seg1_it != seg1.end() && seg2_it != seg2.end()) {
            v_it = (*poly_it).bottom_vertex();
            corner_t vbottom = {(*v_it).x(), (*v_it).y(), 0.};
            if (std::find(seg1.begin(), seg1.end(), vbottom) != seg1.end())
                region.push_back({seg1, seg2});
            else
                region.push_back({seg2, seg1});
        } else if (seg1_it != seg1.end()) {
            region.push_back({seg2, seg1});
        } else {
            region.push_back({seg1, seg2});
        }
    }
    return region;
}

void TrajectoryOptimizer::calcSlopes(const region_t& region,
        std::vector<seg_t>* lowers, std::vector<seg_t>* uppers) {
    lowers->clear();
    uppers->clear();
    for (auto bd : region) {
        seg_t lower, upper;
        border_t::iterator it;

        it = bd.lower.begin();
        for (size_t i(0); i < bd.lower.size() - 1; i++) {
            double m;
            double dely = (*std::next(it, 1)).at(1) - (*it).at(1);
            double delx = (*std::next(it, 1)).at(0) - (*it).at(0);
            if (delx == 0.)
                m = DBL_MAX;
            else
                m = dely / delx;

            double len = pow(pow(delx, 2.0) + pow(dely, 2.0), 0.5);
            lower.push_back(edge_t(*it, edge_prop_t({m, len})));
            it++;
        }
        lowers->push_back(lower);

        it = bd.upper.begin();
        for (int i(0); i < bd.upper.size() - 1; i++) {
            double m;
            double dely = (*std::next(it, 1)).at(1) - (*it).at(1);
            double delx = (*std::next(it, 1)).at(0) - (*it).at(0);
            if (delx == 0.)
                m = DBL_MAX;
            else
                m = dely / delx;

            double len = pow(pow(delx, 2.0) + pow(dely, 2.0), 0.5);
            upper.push_back(edge_t(*it, edge_prop_t({m, len})));
            it++;
        }
        uppers->push_back(upper);
    }
}

void TrajectoryOptimizer::plot(traj_t *traj, const std::string title,
        const std::string xlab,    const std::string ylab,
        double tmin, double tmax, double xmin, double xmax) {
    Gnuplot gp;
    lines_t lines;
    std::string gpcmd;
    if (!(traj->empty())) {
        size_t n = (*(*traj).begin()).second.size();
        bool setTMin(tmin == DBL_MAX), setTMax(tmax == DBL_MIN),
                setXMin(xmin == DBL_MAX), setXMax(xmax == DBL_MIN);

        for (size_t i(0); i < n ; i++)
            lines.push_back(line_t());

        for (auto elem : *traj) {
            double t = elem.first;
            if (tmin > t && setTMin)
                tmin = t -1;
            if (tmax < t && setTMax)
                tmax = t +1;

            int i = 0;
            for (double data : elem.second) {
                lines.at(i++).push_back(std::make_pair(t, data));
                if (xmin > data && setXMin)
                    xmin = data - 1;
                if (xmax < data && setXMax)
                    xmax = data +1;
            }
        }
        gp << "set title '" + title + "'\n";
        gp << "set xlabel'" + xlab  + "'\nset ylabel '" + ylab + "'\n";

        gpcmd = "set xrange [" +
                std::to_string(tmin) + ":"+    std::to_string(tmax) +
                "]\n" + "set yrange [" +
                std::to_string(xmin) + ":"+ std::to_string(xmax) + "]\n";
        gp << gpcmd;

        gp << "set key right bottom\n";

        gpcmd = "plot ";
        for (size_t i(0); i < n ; i++) {
            gpcmd = gpcmd + "'-' with linespoint title 'plot" +
                    std::to_string(i) + "'";
            if (i < n -1)
                gpcmd = gpcmd + ",";
            else
                gpcmd = gpcmd + "\n";
        }
        gp << gpcmd;
        for (size_t i(0); i < n ; i++)
            gp.send1d(lines.at(i));
    } else {
        std::cout << "No Data to Plot!!!" << std::endl;
    }
}

void TrajectoryOptimizer::plotX(const size_t idx) {
    Gnuplot gp;
    if (!(getXtraj()->empty())) {
    	const std::vector<size_t> idxs{idx+1};
    	const ETOL::traj_t &xtraj = *getXtraj();
    	auto traj = TrajectoryOptimizer::extractTraj(xtraj, idxs);
    	std::string title =  "State " + std::to_string(idx);
    	TrajectoryOptimizer::plot(&traj, title, "time", "Value");
    }
}

void TrajectoryOptimizer::plotU(const size_t idx) {
    if (!(getUtraj()->empty())) {
    	const std::vector<size_t> idxs{idx+1};
    	const ETOL::traj_t &utraj = *getUtraj();
    	auto traj = TrajectoryOptimizer::extractTraj(utraj, idxs);
    	std::string title =  "Control " + std::to_string(idx);
    	TrajectoryOptimizer::plot(&traj, title, "time", "Value");
    }
}

void TrajectoryOptimizer::plotXY(traj_t* traj, size_t xIdx, size_t yIdx,
                            const std::string title,  const std::string xlab,
                            const std::string ylab, double xmin, double xmax,
                            double ymin, double ymax) {
    Gnuplot gp;
    if (!(traj->empty())) {
        size_t n = (*(*traj).begin()).second.size();
        bool setXMin(xmin == DBL_MAX), setXMax(xmax == DBL_MIN),
                setYMin(ymin == DBL_MAX), setYMax(ymax == DBL_MIN);

        line_t line;

        for (auto elem : *traj) {
            double x = elem.second.at(xIdx);

            if (xmin > x && setXMin)
                xmin = x;
            if (xmax < x && setXMax)
                xmax = x;

            double y = elem.second.at(yIdx);
            int i = 0;
            if (ymin > y && setYMin)
                ymin = y;
            if (ymax < y && setYMax)
                ymax = y;

            line.push_back(std::make_pair(x, y));
        }
        double xsep = 0.1 * (xmax - xmin);
        double ysep = 0.1 * (ymax - ymin);

        if (setXMin) xmin-= xsep;
        if (setXMax) xmax+= xsep;
        if (setYMin) ymin-= ysep;
        if (setYMax) ymax+= ysep;

        gp << "set title '" + title + "'\n";
        gp << "set xlabel'" + xlab  + "'\nset ylabel '" + ylab + "'\n";
        gp << "set xrange [" +
                std::to_string(xmin) + ":"+    std::to_string(xmax) +
                "]\n" + "set yrange [" +
                std::to_string(ymin) + ":"+ std::to_string(ymax) + "]\n";
        gp << "set key right bottom\n";
        gp << "plot '-' with linespoint\n ";
        gp.send1d(line);
    } else {
        std::cout << "No Data to Plot!!!" << std::endl;
    }
}

void TrajectoryOptimizer::plotXY_wExclZones(traj_t* traj,
            std::list<region_t>* zones,
            size_t xIdx, size_t yIdx,
            const std::string title,
            const std::string xlab,
            const std::string ylab,
            double xmin, double xmax,
            double ymin , double ymax) {
    Gnuplot gp;
    std::string gpcmd;
    if (!(traj->empty())) {
        size_t n = (*(*traj).begin()).second.size();
        bool setXMin(xmin == DBL_MAX), setXMax(xmax == DBL_MIN),
                setYMin(ymin == DBL_MAX), setYMax(ymax == DBL_MIN);

        line_t line;

        for (auto elem : *traj) {
            double x = elem.second.at(xIdx);

            if (xmin > x && setXMin)
                xmin = x;
            if (xmax < x && setXMax)
                xmax = x;

            double y = elem.second.at(yIdx);
            int i = 0;
            if (ymin > y && setYMin)
                ymin = y;
            if (ymax < y && setYMax)
                ymax = y;

            line.push_back(std::make_pair(x, y));
        }
        double xsep = 0.1 * (xmax - xmin);
        double ysep = 0.1 * (ymax - ymin);

        if (setXMin) xmin-= xsep;
        if (setXMax) xmax+= xsep;
        if (setYMin) ymin-= ysep;
        if (setYMax) ymax+= ysep;

        std::list<line_t> obs_lower;
        std::list<line_t> obs_upper;
        if (zones != NULL) {
            if  (!zones->empty()) {
                for (auto zone : *zones) {
                    for (auto bd : zone) {
                        line_t l;
                        line_t u;
                        for_each(bd.lower.begin(), bd.lower.end(),
                                    [&l](const corner_t &pt){
                            l.push_back(std::make_pair(pt.at(0), pt.at(1)));
                        });

                        for_each(bd.upper.begin(), bd.upper.end(),
                                [&u](const corner_t &pt){
                            u.push_back(std::make_pair(pt.at(0), pt.at(1)));
                        });
                        obs_lower.push_back(l);
                        obs_upper.push_back(u);
                    }
                }
            }
        }
        gp << "set title '" + title + "'\n";
        gp << "set xlabel'" + xlab  + "'\nset ylabel '" + ylab + "'\n";
        gp << "set xrange [" +
                std::to_string(xmin) + ":"+    std::to_string(xmax) +
                "]\n" + "set yrange [" +
                std::to_string(ymin) + ":"+ std::to_string(ymax) + "]\n";
        gp << "set key right bottom\n";
        gp << "plot '-' with linespoint title 'Trajectory'";

        for (size_t i(0); i < (obs_lower.size()+ obs_upper.size()); i++) {
            gpcmd+= ",'-' with lines";
        }
        gpcmd = gpcmd + "\n";

        gp << gpcmd;

        gp.send1d(line);
        for (auto obs : obs_lower)
            gp.send1d(obs);
        for (auto obs : obs_upper)
            gp.send1d(obs);
    } else {
        std::cout << "No Data to Plot!!!" << std::endl;
    }
}

std::string TrajectoryOptimizer::animate2D(traj_t* traj, const int framerate,
        bool toFile, std::string outFile,
        std::list<region_t>* obstacles, std::list<track_t>* tracks,
        size_t xIdx, size_t yIdx,
        const std::string title,
        const std::string xlab,
        const std::string ylab,
        double xmin, double xmax,
        double ymin , double ymax) {
    int rc(0);
    char outfile[100];
    std::string cmd(""), gpcmd("");
    if (toFile)
        rc = system("mkdir animation");
    if (rc < 0) {
        printf("animate2D: Error at system 'mkdir'\n");
        return "";
    }
    // Braces ensure gp is collected by garbage collector before 'rm' is called
    {
        Gnuplot gp;

        if (toFile)
            gp << "set terminal png\n";

        if (!(traj->empty())) {
            size_t n = (*(*traj).begin()).second.size();
            bool setXMin(xmin == DBL_MAX), setXMax(xmax == DBL_MIN),
                    setYMin(ymin == DBL_MAX), setYMax(ymax == DBL_MIN);

            line_t line = {};

            std::vector<line_t> mexz_vec = {};
            for (auto elem : *traj) {
                double t = elem.first;
                double x = elem.second.at(xIdx);

                if (xmin > x && setXMin)
                    xmin = x;
                if (xmax < x && setXMax)
                    xmax = x;

                double y = elem.second.at(yIdx);
                int i = 0;
                if (ymin > y && setYMin)
                    ymin = y;
                if (ymax < y && setYMax)
                    ymax = y;

                line.push_back(std::make_pair(x, y));
                if (tracks != NULL) {
                    for (auto track : *tracks) {
                        std::vector<double> tvec = {};
                        for_each(track.trajectory.begin(),
                                track.trajectory.end(),
                                [&tvec](const traj_elem_t &data) {
                            double value = data.first;
                            tvec.push_back(value);
                        });
                        std::vector<double>::iterator tlo_it;
                        tlo_it = std::lower_bound(tvec.begin(), tvec.end(), t);
                        if (tlo_it != tvec.begin())
                            tlo_it = std::prev(tlo_it);
                        size_t jj = distance(tvec.begin(), tlo_it);

                        traj_t::iterator  curr, next;
                        curr = track.trajectory.begin();
                        std::advance(curr, jj);
                        next = curr;
                        if (tlo_it != prev(tvec.end()))
                            std::advance(next, 1);

                        double mx(0), my(0);
                        double delt =  next->first - curr->first;
                        double valx0 = curr->second.at(xIdx);
                        double valy0 = curr->second.at(yIdx);
                        double valx1 = next->second.at(xIdx);
                        double valy1 = next->second.at(yIdx);
                        if (delt != 0) {
                            mx = (valx1 - valx0)/ delt;
                            my = (valy1 - valy0)/ delt;
                        }
                        double cx = valx0 + mx*(t - *tlo_it);
                        double cy = valy0 + my*(t - *tlo_it);
                        double r = track.radius;
                        line_t mexz;
                        for (double alpha=0.; alpha < 1; alpha+=1./24.) {
                            double theta = alpha*2.*3.14159;
                            mexz.push_back(std::make_pair(r*cos(theta) + cx,
                                                           r*sin(theta) + cy));
                        }
                        mexz_vec.push_back(mexz);
                    }
                }
            }
            double xsep = 0.1 * (xmax - xmin);
            double ysep = 0.1 * (ymax - ymin);

            if (setXMin) xmin-= xsep;
            if (setXMax) xmax+= xsep;
            if (setYMin) ymin-= ysep;
            if (setYMax) ymax+= ysep;

            gp << "set title '" + title + "'\n";
            gp << "set xlabel'" + xlab  + "'\nset ylabel '" + ylab + "'\n";
            gp << "set xrange [" +
                    std::to_string(xmin) + ":"+    std::to_string(xmax) +
                    "]\n" + "set yrange [" +
                    std::to_string(ymin) + ":"+ std::to_string(ymax) + "]\n";
            gp << "set key right bottom\n";

            std::list<line_t> obs;
            if (obstacles != NULL) {
                if  (!obstacles->empty()) {
                    for (auto zone : *obstacles)
                        for (auto bd : zone) {
                            line_t l;
                            line_t u;
                            for_each(bd.lower.begin(), bd.lower.end(),
                                        [&l](const corner_t &pt){
                                l.push_back(std::make_pair(pt.at(0), pt.at(1)));
                            });

                            for_each(bd.upper.begin(), bd.upper.end(),
                                    [&u](const corner_t &pt){
                                u.push_back(std::make_pair(pt.at(0), pt.at(1)));
                            });

                            obs.push_back(l);
                            obs.push_back(u);
                        }
                }
            }

            gpcmd = "";
            for (size_t i(0); i < (obs.size() + tracks->size()); i++)
                gpcmd+= ",'-' with lines notitle";

            gpcmd = gpcmd + "\n";

            std::vector<line_t>::iterator mexz_it = mexz_vec.begin();
            for (size_t i = 0; i < line.size(); i++) {
                if (toFile) {
                    snprintf(outfile, sizeof(outfile),
                            "'animation/my_graph_%03zu.png'", i);
                    gp << "set output " + std::string(outfile) + "\n";
                }

                line_t trajline(line.begin(), line.begin()+i+1);

                gp << "plot '-' with linespoint title 'Trajectory'";

                gp << gpcmd;

                gp.send1d(trajline);


                for (auto o : obs)
                    gp.send1d(o);

                if (tracks != NULL) {
                    for (auto track : *tracks) {
                        gp.send1d(*mexz_it);
                        std::advance(mexz_it, 1);
                    }
                }

                gp.flush();
                if (toFile)
                    mysleep(10);
                else
                    mysleep((useconds_t)(1000.0 /
                            static_cast<double>(framerate)));
            }
        } else {
            std::cout << "No Data to Animate!!!" << std::endl;
            outFile = "";
        }
    }
    if (toFile) {
        if (outFile == "")
            outFile = "animation.mp4";
        cmd = "";
        cmd = cmd + "ffmpeg -r "+ std::to_string(framerate) +
                " -f image2 -s 1920x1080 -i animation/my_graph_%03d.png "+
                "-vcodec libx264 -crf 25  -pix_fmt yuv420p " + outFile;
        std::cout << cmd << std::endl;
        rc = system(cmd.c_str());
        if (rc < 0) {
            printf("animate2D: Error at system 'ffmpeg'\n");
            return "";
        }
        mysleep(10);
        rc = system("rm -rf animation");
        if (rc < 0) {
            printf("animate2D: Error at system 'rm'\n");
            return "";
        }
    }
    return outFile;
}

std::string TrajectoryOptimizer::save(traj_t* traj, std::string fp) {
    if (!(traj->empty())) {
        std::string ext = fp.substr(fp.find('.'));
        // Do not overwrite file, but increment with trailing number
        struct stat buf;
        while (stat(fp.c_str(), &buf) != -1) {
            int fIdx;
            std::string fname = fp.substr(0, fp.find('.'));
            size_t last_index = fname.find_last_not_of("0123456789")+1;
            if (fname.size() == last_index)
                fIdx = 0;
            else
                fIdx = std::atoi(fname.substr(last_index).c_str());
            fp = fname.substr(0, last_index) + std::to_string(++fIdx) + ext;
        }

        std::ofstream f;
        f.open(fp, std::ios::out);
        size_t n = (*(*traj).begin()).second.size();

        // Write column labels
        std::string row = "time";
        for (size_t idx(0); idx < n; idx++) {
            row+= ",traj" + std::to_string(idx);
        }
        row+= "\n";

        f << row;

        size_t i(0);
        size_t length = (*traj).size();
        // Write data
        for (auto elem : *traj) {
            row = std::to_string(elem.first);
            for (auto data : elem.second) {
                row+= "," + std::to_string(data);
            }
            if (++i != length)
                row+= "\n";

            f << row;
        }
        f.close();
    } else {
        std::cout << "No Data to Save!!!" << std::endl;
    }

    return fp;
}

// API

void TrajectoryOptimizer::resetConfigs() {
    this->setNStates(0);
    this->setNControls(0);
    this->setDt(0);

    this->setXrhorizon(0);
    this->_xvartype.clear();
    this->_x0.clear();
    this->_xf.clear();
    this->_xtol.clear();

    this->setUrhorizon(0);
    this->_uvartype.clear();
    this->_ulower.clear();
    this->_uupper.clear();

    this->_obstacles_raw.clear();
    this->_obstacles.clear();
    this->_tracks.clear();
}

void TrajectoryOptimizer::printConfigs() {
    using std::cout;
    using std::endl;
    cout << endl << "ETOL Information" << endl;
    cout << "# Steps:\t" << this->getNSteps() <<endl;
    cout << "dt:\t\t" << this->getDt() <<endl;
    cout << "# States:\t" << this->getNStates() <<endl;
    cout << "# Controls:\t" << this->getNControls() <<endl;
    cout << "Xrhorizon:\t" << this->getXrhorizon() <<endl;
    cout << "Urhorizon:\t" << this->getUrhorizon() <<endl;
    cout << "#Obstacles:\t" << this->getNExclZones() <<endl;
    cout << "#Tracks:\t" << this->getNTracks() <<endl;
    cout << endl;
    size_t i(0);
    cout <<"\t\tvartype\tlower\tupper\tinitial\tfinal\ttol"    <<endl;
    for (auto vartype : this->_xvartype) {
        char vtype;
        switch (vartype) {
        case var_t::CONTINUOUS:
            vtype = 'C';
            break;
        case var_t::BINARY:
            vtype = 'B';
            break;
        case var_t::INTERGER:
            vtype = 'I';
            break;
        default:
            break;
        }
        cout << "XInfo:\t\t" << vtype << "\t" << this->_xlower.at(i) << "\t" <<
                this->_xupper.at(i) << "\t" << this->_x0.at(i) << "\t"
                << this->_xf.at(i) << "\t" << this->_xtol.at(i) << endl;
        i++;
    }
    i = 0;
    for (auto vartype : this->_uvartype) {
        char vtype;
        switch (vartype) {
        case var_t::CONTINUOUS:
            vtype = 'C';
            break;
        case var_t::BINARY:
            vtype = 'B';
            break;
        case var_t::INTERGER:
            vtype = 'I';
            break;
        default:
            break;
        }
        cout <<"UInfo:\t\t" << vtype <<"\t" << this->_ulower.at(i)
                << "\t"    << this->_uupper.at(i) << endl;
        i++;
    }
    cout << endl;
    cout << "\t\tExclusion Zone Corners..." <<endl;
    for (auto obs : *this->getObstacles()) {
        for (auto bd : obs) {
            cout << "Bottom\t\t";
            for (auto corner : bd.lower)
                cout << "(" << corner.at(0) << "," << corner.at(1)  << ","
                        <<corner.at(2) << ")  ";

            cout << endl;
            cout << "Top\t\t";
            for (auto corner : bd.upper)
                cout << "(" << corner.at(0) << "," << corner.at(1)  << ","
                                        <<corner.at(2) << ")  ";
            cout << endl;
        }
        cout << endl;
    }
    for (auto track : *this->getTracks()) {
        cout << "\t\tradius\t#points"<< endl;
        cout << "TrackInfo\t" << track.radius <<
                "\t" << track.trajectory.size() <<endl << endl;
        cout << "\t\ttime\tElements..." << endl;
        for (auto elem : track.trajectory) {
            cout << "Waypoint\t" <<elem.first << "\t";
            for (auto datum : elem.second)
                cout << datum  << "\t";
            cout << endl;
        }
        cout << endl;
    }
}

void TrajectoryOptimizer::loadConfigs(const char* filepath) {
    using std::string;

    resetConfigs();

    LIBXML_TEST_VERSION
    xmlDoc *doc = xmlutils::getdoc(string(filepath));

    xmlChar* rootPath = reinterpret_cast<xmlChar*>(
                            const_cast<char*>("//etol"));
    xmlXPathObjectPtr root = xmlutils::getnodeset(doc, rootPath);


    {
        xmlAttrPtr attr = root->nodesetval->nodeTab[0]->properties;
        while (attr) {
            string  nname = string((const char*)attr->name);
            xmlChar* val = xmlNodeListGetString(
                                attr->doc, attr->children, 1);

            if (nname == "nsteps")
                this->setNSteps((size_t) xmlXPathCastStringToNumber(val));
            else if (nname == "dt")
                this->setDt(xmlXPathCastStringToNumber(val));

            attr = attr->next;
            xmlFree(val);
        }
        xmlFree(attr);
    }
    assert(this->getNSteps() != 0);
    assert(this->getDt() != 0);


    xmlNodePtr nodes = root->nodesetval->nodeTab[0]->children;
    while (nodes) {
        string nname = string((const char*)nodes->name);
        size_t nstates = 0;
        // State Data
        if (nname == "states") {
            {
                xmlAttrPtr attr = nodes->properties;
                while (attr) {
                    string nname = string((const char*)attr->name);
                    xmlChar* val = xmlNodeListGetString(
                                    attr->doc, attr->children, 1);

                    if (nname == "nstates")
                        nstates =    (size_t) xmlXPathCastStringToNumber(val);
                    else if (nname == "rhorizon")
                        setXrhorizon(std::max(getXrhorizon(), (size_t)
                                xmlXPathCastStringToNumber(val)));
                    attr = attr->next;
                    xmlFree(val);
                }
                xmlFree(attr);
            }
            xmlNodePtr state = nodes->children;
            while (state) {
                if (nstates > this->getNStates()) {
                    xmlAttrPtr attr = state->properties;
                    while (attr) {
                        string nname = string((const char*)attr->name);
                        xmlChar* val = xmlNodeListGetString(
                                attr->doc, attr->children, 1);

                        if (nname == "vartype") {
                            this->setNStates(this->getNStates() + 1);
                            switch (*reinterpret_cast<char*>(val)) {
                            case 'C':
                                this->_xvartype.push_back(var_t::CONTINUOUS);
                                break;
                            case 'B':
                                this->_xvartype.push_back(var_t::BINARY);
                                break;
                            case 'I':
                                this->_xvartype.push_back(var_t::INTERGER);
                                break;
                            default:
                                std::cout << "Invalid xVartype" << std::endl;
                                exit(EXIT_FAILURE);
                            }
                        } else if (nname == "lower") {
                            _xlower.push_back(xmlXPathCastStringToNumber(val));
                        } else if (string((const char*)attr->name) == "upper") {
                            _xupper.push_back(xmlXPathCastStringToNumber(val));
                        } else if (string((const char*)attr->name)
                                                                == "initial") {
                            _x0.push_back(xmlXPathCastStringToNumber(val));
                        } else if (string((const char*)attr->name)
                                                                == "terminal") {
                            _xf.push_back(xmlXPathCastStringToNumber(val));
                        } else if (string((const char*)attr->name)
                                                            == "tolerance") {
                            _xtol.push_back(xmlXPathCastStringToNumber(val));
                        }

                        attr = attr->next;
                        xmlFree(val);
                    }
                    xmlFree(attr);
                }
                state = state->next;
            }
            xmlFree(state);
        } else if (nname == "controls") {
            size_t ncontrols = 0;
            {
                xmlAttrPtr attr = nodes->properties;
                while (attr) {
                    string nname = string((const char*)attr->name);
                    xmlChar* val = xmlNodeListGetString(
                            attr->doc, attr->children, 1);
                    if (nname == "ncontrols")
                        ncontrols = (size_t) xmlXPathCastStringToNumber(val);
                    else if (nname == "rhorizon")
                        setUrhorizon(std::max(getUrhorizon(), (size_t)
                                xmlXPathCastStringToNumber(val)));
                    attr = attr->next;
                    xmlFree(val);
                }
                xmlFree(attr);
            }
            xmlNodePtr control =  nodes->children;
            while (control) {
                if (ncontrols > this->getNControls()) {
                    xmlAttrPtr attr = control->properties;
                    while (attr) {
                        string nname = string((const char*)attr->name);
                        xmlChar* val = xmlNodeListGetString(
                                attr->doc, attr->children, 1);
                        if (nname == "vartype") {
                            this->setNControls(this->getNControls() + 1);
                            switch (*reinterpret_cast<char*>(val)) {
                            case 'C':
                                this->_uvartype.push_back(var_t::CONTINUOUS);
                                break;
                            case 'B':
                                this->_uvartype.push_back(var_t::BINARY);
                                break;
                            case 'I':
                                this->_uvartype.push_back(var_t::INTERGER);
                                break;
                            default:
                                std::cout << "Invalid uVartype" << std::endl;
                                exit(EXIT_FAILURE);
                            }
                        } else if (string((const char*)attr->name) == "lower") {
                            _ulower.push_back(xmlXPathCastStringToNumber(val));
                        } else if (string((const char*)attr->name) == "upper") {
                            _uupper.push_back(xmlXPathCastStringToNumber(val));
                        }

                        attr = attr->next;
                        xmlFree(val);
                    }
                    xmlFree(attr);
                }
                control = control->next;
            }
            xmlFree(control);
        } else if (nname == "exzones") {
            size_t nexzones(SIZE_MAX);
            {
                xmlAttrPtr attr = nodes->properties;
                while (attr) {
                    string nname = string((const char*)attr->name);
                    xmlChar* val = xmlNodeListGetString(
                            attr->doc, attr->children, 1);
                    if (nname == "nzones")
                        nexzones = (size_t) xmlXPathCastStringToNumber(val);
                    attr = attr->next;
                    xmlFree(val);
                }
                xmlFree(attr);
            }
            size_t i(0);
            xmlNodePtr border =  nodes->children;
            while (border) {
                size_t ncorners(SIZE_MAX);
                {
                    xmlAttrPtr attr = border->properties;
                    while (attr) {
                        string nname = string((const char*)attr->name);
                        xmlChar* val = xmlNodeListGetString(
                                attr->doc, attr->children, 1);

                        if (nname == "ncorners")
                            ncorners = (size_t) xmlXPathCastStringToNumber(val);

                        attr = attr->next;
                        xmlFree(val);
                    }
                    xmlFree(attr);
                }
                border_t border_val = border_t();
                xmlNodePtr corner = border->children;
                while (corner) {
                    xmlAttrPtr attr = corner->properties;
                    double x(DBL_MIN), y(DBL_MIN), z(DBL_MIN);
                    while (attr) {
                        string nname = string((const char*)attr->name);
                        xmlChar* val = xmlNodeListGetString(
                                        attr->doc, attr->children, 1);
                        if (nname == "x")
                            x = xmlXPathCastStringToNumber(val);
                        else if (nname == "y")
                            y = xmlXPathCastStringToNumber(val);
                        else if (nname == "z")
                            z = xmlXPathCastStringToNumber(val);

                        attr = attr->next;
                        xmlFree(val);
                    }
                    if (x != DBL_MIN && y != DBL_MIN && z != DBL_MIN &&
                            !(border_val.size() > ncorners)) {
                        xmlFree(attr);
                        corner_t corner_val {x, y, z};
                        border_val.push_back(corner_val);
                    }
                    corner = corner->next;
                }
                xmlFree(corner);
                if (!border_val.empty() && (nexzones > i)) {
                    this->addExclZone(&border_val);
                    i++;
                }

                border = border->next;
            }
            xmlFree(border);
        } else if (nname == "mexzones") {
            double nmexzones = 0;
            {
                xmlAttrPtr attr = nodes->properties;
                while (attr) {
                    string nname = string((const char*)attr->name);
                    xmlChar* val = xmlNodeListGetString(
                                    attr->doc, attr->children, 1);
                    if (nname == "nzones")
                        nmexzones = (size_t) xmlXPathCastStringToNumber(val);

                    attr = attr->next;
                    xmlFree(val);
                }
                xmlFree(attr);
            }
            size_t i = 0;
            xmlNodePtr mexzone = nodes->children;
            while (mexzone) {
                size_t nway = 0;
                track_t track = track_t();
                {
                    xmlAttrPtr attr = mexzone->properties;
                    while (attr) {
                        string nname = string((const char*)attr->name);
                        xmlChar* val = xmlNodeListGetString(
                                attr->doc, attr->children, 1);
                        if (nname == "radius")
                            track.radius = xmlXPathCastStringToNumber(val);
                        else if (nname == "nwaypoints")
                            nway = (size_t) xmlXPathCastStringToNumber(val);

                        attr = attr->next;
                        xmlFree(val);
                    }
                    xmlFree(attr);
                }
                size_t j(0);
                traj_t traj;
                xmlNodePtr waypoint = mexzone->children;
                while (waypoint) {
                    size_t ndatums = 0;
                    traj_elem_t traj_elem;
                    double t = 0.;
                    {
                        xmlAttrPtr attr = waypoint->properties;
                        while (attr) {
                            string nname = string((const char*) attr->name);
                            xmlChar* val = xmlNodeListGetString(
                                            attr->doc, attr->children, 1);
                            if (nname == "t")
                                traj_elem.first =
                                        xmlXPathCastStringToNumber(val);
                            else if (nname == "ndatums")
                                ndatums = (size_t)
                                            xmlXPathCastStringToNumber(val);

                            attr = attr->next;
                            xmlFree(val);
                        }
                        xmlFree(attr);
                    }

                    state_t state;
                    xmlNodePtr datum = waypoint->children;
                    while (datum) {
                        string nname = string((const char*) datum->name);
                        xmlChar* val = xmlNodeListGetString(
                                        datum->doc, datum->children, 1);

                        if (nname == "datum" &&
                                !(ndatums !=0 && state.size() >= ndatums))
                            state.push_back(
                                    xmlXPathCastStringToNumber(val));
                        xmlFree(val);
                        datum = datum->next;
                    }
                    xmlFree(datum);
                    if (!state.empty() && (nway > traj.size())) {
                        traj_elem.second = state;
                        traj.push_back(traj_elem);
                    }
                    waypoint = waypoint->next;
                }
                xmlFree(waypoint);

                if (!traj.empty() && (nmexzones > this->getTracks()->size())) {
                    track.trajectory = traj;
                    this->addAdjTrack(&track);
                }
                mexzone = mexzone->next;
            }
            xmlFree(mexzone);
        }
        nodes = nodes->next;
    }
    xmlFree(nodes);
    xmlXPathFreeObject(root);
    xmlFreeDoc(doc);
}

void TrajectoryOptimizer::saveConfigs(const char* filepath) {
    /*
     * this initialize the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    using std::to_string;
    LIBXML_TEST_VERSION
    int rc;
    xmlTextWriterPtr writer;
    xmlChar *tmp;
    size_t i, j, nzones;
    char str[MAXLENGTH];

    /* Create a new XmlWriter for uri, with no compression. */
    writer = xmlNewTextWriterFilename(filepath, 0);
    if (writer == NULL) {
        printf("testXmlwriterFilename: Error creating the xml writer\n");
        return;
    }
    rc = xmlTextWriterSetIndent(writer, 1);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterSetIndent\n");
        return;
    }

    memset(str, '\0', 10*sizeof(char));
    snprintf(str, sizeof(str), "\t");
    rc = xmlTextWriterSetIndentString(writer, BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterSetIndent\n");
        return;
    }

    // Start the document with the xml default for the version,
    // encoding and the default for the standalone declaration.
    rc = xmlTextWriterStartDocument(writer, NULL, MY_ENCODING, NULL);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterStartDocument\n");
        return;
    }

    // Make ETOL the root node of the tree
    rc = xmlTextWriterStartElement(writer, BAD_CAST "etol");
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterStartElement\n");
        return;
    }

    memset(str, '\0', 10*sizeof(char));
    snprintf(str, sizeof(str), "%zu", this->getNSteps());
    rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "nsteps", BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
        return;
    }

    memset(str, '\0', 10*sizeof(char));
    snprintf(str, sizeof(str), "%.02f", this->getDt());
    rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "dt", BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
        return;
    }

    // Start an element named "states" as child of ETOL.
    rc = xmlTextWriterStartElement(writer, BAD_CAST "states");
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterStartElement\n");
        return;
    }

    memset(str, '\0', 10*sizeof(char));
    snprintf(str, sizeof(str), "%zu", this->getNStates());
    rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "nstates", BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
        return;
    }

    memset(str, '\0', 10*sizeof(char));
    snprintf(str, sizeof(str), "%zu", this->getXrhorizon());
    rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "rhorizon", BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
        return;
    }

    for (i =0 ; i < this->getNStates(); i++) {
        // Start an element named "states" as child of ETOL.
        rc = xmlTextWriterStartElement(writer, BAD_CAST "state");
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterStartElement\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%s",
                    std::string("x" + to_string(i)).c_str());
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        char vtype = 'C';
        switch (this->getXvartype().at(i))    {
            case var_t::CONTINUOUS:
                vtype = 'C';
                break;
            case var_t::BINARY:
                vtype = 'B';
                break;
            case var_t::INTERGER:
                vtype = 'I';
                break;
            default:
                break;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%c", vtype);
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "vartype",
                                            BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%.2f", getXlower().at(i));
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "lower",
                                            BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%.2f", getXupper().at(i));
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "upper",
                                            BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%.2f", this->getX0().at(i));
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "initial",
                                            BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%.2f", this->getXf().at(i));
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "terminal",
                                                    BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%.2f", this->getXtol().at(i));
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "tolerance",
                                                 BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        // Close the element named state.
        rc = xmlTextWriterEndElement(writer);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterEndElement\n");
            return;
        }
    }

    // Close the element named states.
    rc = xmlTextWriterEndElement(writer);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterEndElement\n");
        return;
    }


    // Start an element named "states" as child of ETOL.
    rc = xmlTextWriterStartElement(writer, BAD_CAST "controls");
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterStartElement\n");
        return;
    }

    memset(str, '\0', 10*sizeof(char));
    snprintf(str, sizeof(str), "%zu", this->getNControls());
    rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "ncontrols",
                                        BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
        return;
    }

    memset(str, '\0', 10*sizeof(char));
    snprintf(str, sizeof(str), "%zu", this->getUrhorizon());
    rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "rhorizon", BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
        return;
    }

    for (i =0; i < this->getNStates(); i++) {
        // Start an element named "controls" as child of ETOL.
        rc = xmlTextWriterStartElement(writer, BAD_CAST "control");
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterStartElement\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%s",
                    std::string("u" + to_string(i)).c_str());
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        char vtype = 'C';
        switch (this->getUvartype().at(i))    {
            case var_t::CONTINUOUS:
                vtype = 'C';
                break;
            case var_t::BINARY:
                vtype = 'B';
                break;
            case var_t::INTERGER:
                vtype = 'I';
                break;
            default:
                break;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%c", vtype);
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "vartype",
                                            BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%.2f", getUlower().at(i));
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "lower",
                                            BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        memset(str, '\0', 10*sizeof(char));
        snprintf(str, sizeof(str), "%.2f", getUupper().at(i));
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "upper",
                                            BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        // Close the element named control.
        rc = xmlTextWriterEndElement(writer);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterEndElement\n");
            return;
        }
    }

    // Close the element named controls.
    rc = xmlTextWriterEndElement(writer);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterEndElement\n");
        return;
    }

    // Start an element named "exzones" as child of ETOL.
    rc = xmlTextWriterStartElement(writer, BAD_CAST "exzones");
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterStartElement\n");
        return;
    }

    nzones = 0;
    for (region_t obs : *this->getObstacles())
        nzones +=obs.size();
    memset(str, '\0', 10*sizeof(char));
    snprintf(str, sizeof(str), "%zu", nzones);
    rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "nzones", BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
        return;
    }

    i = 0;
    for (region_t obs : *this->getObstacles()) {
        for (auto bd : obs) {
            border_t corners;
            // Start an element named "border" as child of exzones.
            rc = xmlTextWriterStartElement(writer, BAD_CAST "border");
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterStartElement\n");
                return;
            }
            memset(str, '\0', 10*sizeof(char));
            snprintf(str, sizeof(str), "%s",
                    std::string("exz" + to_string(i)).c_str());
            rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "name",
                                                BAD_CAST str);
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
                return;
            }
            for (auto corn : bd.lower) {
                corners.push_back(corn);
            }
            corners.erase(std::prev(corners.end()));
            for_each(bd.upper.rbegin(), bd.upper.rend(),
                    [&corners](corner_t corn) {
                corners.push_back(corn);
            });
            corners.erase(std::prev(corners.end()));
            memset(str, '\0', 10*sizeof(char));
            snprintf(str, sizeof(str), "%zu", corners.size());
            rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "ncorners",
                                                BAD_CAST str);
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
                return;
            }

            for (auto corn : corners) {
                // Start an element named "corner" as child of border.
                rc = xmlTextWriterStartElement(writer, BAD_CAST "corner");
                if (rc < 0) {
                    printf(
                        "saveConfigs: Error at xmlTextWriterStartElement\n");
                    return;
                }
                memset(str, '\0', 10*sizeof(char));
                snprintf(str, sizeof(str), "%.02f", corn.at(0));
                rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "x",
                                                            BAD_CAST str);
                if (rc < 0) {
                    printf(
                        "saveConfigs: Error at xmlTextWriterWriteAttribute\n");
                    return;
                }
                memset(str, '\0', 10 * sizeof(char));
                snprintf(str, sizeof(str), "%.02f", corn.at(1));
                rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "y",
                                                            BAD_CAST str);
                if (rc < 0) {
                    printf(
                        "saveConfigs: Error at xmlTextWriterWriteAttribute\n");
                    return;
                }
                memset(str, '\0', 10 * sizeof(char));
                snprintf(str, sizeof(str), "%.02f", corn.at(2));
                rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "z",
                                                            BAD_CAST str);
                if (rc < 0) {
                    printf(
                        "saveConfigs: Error at xmlTextWriterWriteAttribute\n");
                    return;
                }
                // Close the element named corner.
                rc = xmlTextWriterEndElement(writer);
                if (rc < 0) {
                    printf("saveConfigs: Error at xmlTextWriterEndElement\n");
                    return;
                }
            }
            // Close the element named border.
            rc = xmlTextWriterEndElement(writer);
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterEndElement\n");
                return;
            }
            i++;
        }
    }

    // Close the element named exzones.
    rc = xmlTextWriterEndElement(writer);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterEndElement\n");
        return;
    }

    // Start an element named "mexzones" as child of ETOL.
    rc = xmlTextWriterStartElement(writer, BAD_CAST "mexzones");
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterStartElement\n");
        return;
    }

    nzones = this->getNTracks();
    memset(str, '\0', 10 * sizeof(char));
    snprintf(str, sizeof(str), "%zu", nzones);
    rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "nzones", BAD_CAST str);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
        return;
    }

    i = 0;
    for (auto track : *this->getTracks()) {
        // Start an element named "border" as child of exzones.
        rc = xmlTextWriterStartElement(writer, BAD_CAST "track");
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterStartElement\n");
            return;
        }

        memset(str, '\0', 10 * sizeof(char));
        snprintf(str, sizeof(str), "%s",
                    std::string("mexz" + to_string(i)).c_str());
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "name", BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }

        memset(str, '\0', 10 * sizeof(char));
        snprintf(str, sizeof(str), "%zu", track.trajectory.size());
        rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "nwaypoints",
                                                BAD_CAST str);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
            return;
        }
        j = 0;
        for (auto  traj : track.trajectory) {
            rc = xmlTextWriterStartElement(writer, BAD_CAST "waypoint");
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterStartElement\n");
                return;
            }

            memset(str, '\0', 10 * sizeof(char));
            snprintf(str, sizeof(str), "%s",
                        std::string("pt" + std::to_string(j)).c_str());
            rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "name",
                                            BAD_CAST str);
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
                return;
            }

            memset(str, '\0', 10 * sizeof(char));
            snprintf(str, sizeof(str), "%.2f", traj.first);
            rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "t",
                                                BAD_CAST str);
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
                return;
            }

            memset(str, '\0', 10 * sizeof(char));
            snprintf(str, sizeof(str), "%zu", traj.second.size());
            rc = xmlTextWriterWriteAttribute(writer, BAD_CAST "ndatums",
                                                    BAD_CAST str);
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterWriteAttribute\n");
                return;
            }

            for (auto pt : traj.second) {
                rc = xmlTextWriterWriteFormatElement(writer, BAD_CAST "datum",
                                                         "%.2f", pt);
                if (rc < 0) {
                    printf
                        ("testXmlwriterFilename: Error at WriteElement\n");
                    return;
                }
            }
            // Close the element named waypoint.
            rc = xmlTextWriterEndElement(writer);
            if (rc < 0) {
                printf("saveConfigs: Error at xmlTextWriterEndElement\n");
                return;
            }
            j++;
        }
        // Close the element named track.
        rc = xmlTextWriterEndElement(writer);
        if (rc < 0) {
            printf("saveConfigs: Error at xmlTextWriterEndElement\n");
            return;
        }
        i++;
    }

    // Here we could close the elements, but since we do not want to write any
    // other elements, we simply call xmlTextWriterEndDocument, which will do
    // all the work.
    rc = xmlTextWriterEndDocument(writer);
    if (rc < 0) {
        printf("saveConfigs: Error at xmlTextWriterEndDocument\n");
        return;
    }

    xmlFreeTextWriter(writer);
}

void TrajectoryOptimizer::addParams(std::list<param_t> params) {
    for (auto elem : params)
        this->_parameters.insert(std::make_pair(elem.first, elem.second));
}

void TrajectoryOptimizer::addExclZone(border_t* border) {
    this->_obstacles_raw.push_back(*border);
    region_t obstacle = TrajectoryOptimizer::genRegion(border);
    if (!obstacle.empty())
        this->_obstacles.push_back(obstacle);
}

void TrajectoryOptimizer::addAdjTrack(track_t* track) {
    this->_tracks.push_back(*track);
}

// Getters and Setters

const double TrajectoryOptimizer::getScore() const {
    return _score;
}

void TrajectoryOptimizer::setScore(const double score) {
    _score = score;
}

state_t& TrajectoryOptimizer::getX0() {
    return _x0;
}

void TrajectoryOptimizer::setX0(const state_t& x0) {
    this->_x0 = x0;
}

state_t& TrajectoryOptimizer::getXf() {
    return _xf;
}

void TrajectoryOptimizer::setXf(const state_t& xf) {
    this->_xf = xf;
}

const size_t TrajectoryOptimizer::getNControls() const {
    return _nControls;
}

const size_t TrajectoryOptimizer::getNStates() const {
    return _nStates;
}

state_t& TrajectoryOptimizer::getXlower() {
    return _xlower;
}

void TrajectoryOptimizer::setXlower(const state_t& xlower) {
    _xlower = xlower;
}

state_t& TrajectoryOptimizer::getXupper() {
    return _xupper;
}

void TrajectoryOptimizer::setXupper(const state_t& xupper) {
    _xupper = xupper;
}

state_var_t& TrajectoryOptimizer::getXvartype() {
    return _xvartype;
}

void TrajectoryOptimizer::setXvartype(const state_var_t& xvartype) {
    _xvartype = xvartype;
}

const double TrajectoryOptimizer::getDt() const {
    return _dt;
}

void TrajectoryOptimizer::setDt(const double dt) {
    _dt = dt;
}

const size_t TrajectoryOptimizer::getNSteps() const {
    return _nSteps;
}

void TrajectoryOptimizer::setNSteps(const size_t nSteps) {
    _nSteps = nSteps;
}

state_t& TrajectoryOptimizer::getXtol() {
    return _xtol;
}

void TrajectoryOptimizer::setXtol(const state_t &xtol) {
    _xtol = xtol;
    for_each(_xtol.begin(), _xtol.end(), [](double x){
        return fabs(x);
    });
}

state_t& TrajectoryOptimizer::getUlower() {
    return _ulower;
}

void TrajectoryOptimizer::setUlower(const state_t &ulower) {
    _ulower = ulower;
}

state_t& TrajectoryOptimizer::getUupper() {
    return _uupper;
}

void TrajectoryOptimizer::setUupper(const state_t &uupper) {
    _uupper = uupper;
}

state_var_t& TrajectoryOptimizer::getUvartype() {
    return _uvartype;
}

void TrajectoryOptimizer::setUvartype(const state_var_t &uvartype) {
    _uvartype = uvartype;
}

const size_t TrajectoryOptimizer::getUrhorizon() const {
    return _urhorizon;
}

void TrajectoryOptimizer::setUrhorizon(const size_t urhorizon) {
    _urhorizon = urhorizon;
    this->_rhorizon = std::max(this->_xrhorizon, this->_urhorizon);
}

const size_t TrajectoryOptimizer::getXrhorizon() const {
    return _xrhorizon;
}

void TrajectoryOptimizer::setXrhorizon(const size_t xrhorizon) {
    _xrhorizon = xrhorizon;
    this->_rhorizon = std::max(this->_xrhorizon, this->_urhorizon);
}

size_t TrajectoryOptimizer::getRhorizon() const {
    return _rhorizon;
}

void TrajectoryOptimizer::setNControls(const size_t nControls) {
    _nControls = nControls;
}

void TrajectoryOptimizer::setNStates(const size_t nStates) {
    _nStates = nStates;
}

void TrajectoryOptimizer::setConstraints(std::vector<f_t*> constraints) {
    this->_constraints = constraints;
}

void TrajectoryOptimizer::setEqConstraints(std::vector<f_t*> constraints) {
    this->_eq = constraints;
}

void TrajectoryOptimizer::setLessEqConstraints(std::vector<f_t*> constraints) {
    this->_lesseq = constraints;
}

void TrajectoryOptimizer::setGradient(std::vector<f_t*> gradient) {
    this->_gradient = gradient;
}

void TrajectoryOptimizer::setObjective(f_t* objective) {
    _objective = objective;
}

void TrajectoryOptimizer::errorHandler() {
    if (this->_eAny != NULL) {
        fprintf(stderr, "%s", this->_eAny->what());
        exit(EXIT_FAILURE);
    }
}

traj_t* TrajectoryOptimizer::getUtraj() {
    return &(_utraj);
}

traj_t* TrajectoryOptimizer::getXtraj() {
    return &(_xtraj);
}

const f_t* TrajectoryOptimizer::getObjective() const {
    return _objective;
}

std::vector<f_t*>* TrajectoryOptimizer::getGradient() {
    return &(_gradient);
}

std::vector<f_t*>* TrajectoryOptimizer::getEqConstraints() {
    return &(_eq);
}

std::vector<f_t*>* TrajectoryOptimizer::getLessEqConstraints() {
    return &(_lesseq);
}

std::vector<f_t*>* TrajectoryOptimizer::getConstraints() {
    return &(_constraints);
}

std::vector<border_t>* TrajectoryOptimizer::getObstacles_Raw() {
    return &_obstacles_raw;
}

std::list<region_t>* TrajectoryOptimizer::getObstacles() {
    return &_obstacles;
}

std::list<track_t>* TrajectoryOptimizer::getTracks() {
    return &_tracks;
}

bool TrajectoryOptimizer::isMaximized() const {
    return _maximize;
}

void TrajectoryOptimizer::setMaximize(const bool maximize) {
    _maximize = maximize;
}

size_t TrajectoryOptimizer::getNExclZones() {
    return _obstacles.size();
}

size_t TrajectoryOptimizer::getNTracks() {
    return _tracks.size();
}

} /* namespace ETOL */
