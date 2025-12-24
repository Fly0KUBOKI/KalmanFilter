/* mex_matlab_helpers.cpp
 * Consolidated Phase-1 helper: get_field, has_field, get_euler
 * Single mexFunction and M_PI defined for MSVC environments.
 */

#include <mex.h>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static std::string getCommand(const mxArray* a) {
    if (!mxIsChar(a)) return std::string();
    char buf[512];
    mxGetString(a, buf, sizeof(buf));
    return std::string(buf);
}

static std::string getCellString(const mxArray* cell, mwIndex idx) {
    if (!mxIsCell(cell)) return std::string();
    const mxArray* c = mxGetCell(cell, idx);
    if (!c || !mxIsChar(c)) return std::string();
    char buf[512]; mxGetString(c, buf, sizeof(buf));
    return std::string(buf);
}

void do_get_field(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 5) mexErrMsgIdAndTxt("mex_matlab_helpers:get_field","Usage: mex_matlab_helpers('get_field', obs, fieldsCell, idx, ncols)");
    const mxArray* obs = prhs[1];
    const mxArray* fields = prhs[2];
    double idx_d = mxGetScalar(prhs[3]);
    double ncols_d = mxGetScalar(prhs[4]);
    mwIndex idx = (mwIndex)(idx_d - 1); // MATLAB -> C
    int ncols = (int)ncols_d;

    mwSize n_fields = mxGetNumberOfElements(fields);
    for (mwIndex f = 0; f < n_fields; ++f) {
        std::string fname = getCellString(fields, f);
        if (fname.empty()) continue;
        if (mxGetField(obs, 0, fname.c_str()) != NULL) {
            const mxArray* field = mxGetField(obs, 0, fname.c_str());
            if (ncols == 1) {
                if (mxIsNumeric(field)) {
                    double* data = mxGetPr(field);
                    mwSize ne = mxGetNumberOfElements(field);
                    if (idx < ne) {
                        plhs[0] = mxCreateDoubleScalar(data[idx]);
                        return;
                    }
                }
            } else if (ncols == 3) {
                if (mxIsNumeric(field) && mxGetNumberOfElements(field) >= 3) {
                    mwSize nrow = mxGetM(field);
                    mwSize ncol = mxGetN(field);
                    double* data = mxGetPr(field);
                    if (ncol >= 3 && idx < nrow) {
                        plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
                        double* out = mxGetPr(plhs[0]);
                        out[0] = data[idx + 0*nrow];
                        out[1] = data[idx + 1*nrow];
                        out[2] = data[idx + 2*nrow];
                        return;
                    }
                }
                std::string base = fname;
                if (base.size() > 2 && base.substr(base.size()-2) == "_x") base = base.substr(0, base.size()-2);
                std::vector<std::string> try_names;
                try_names.push_back(base + "_x"); try_names.push_back(base + "_y"); try_names.push_back(base + "_z");
                try_names.push_back(base + "x"); try_names.push_back(base + "y"); try_names.push_back(base + "z");
                bool ok = true;
                double vals[3] = {0,0,0};
                for (int k=0;k<3;k++){
                    const char* nm = try_names[k].c_str();
                    const mxArray* fmx = mxGetField(obs, 0, nm);
                    if (!fmx) { ok = false; break; }
                    if (!mxIsNumeric(fmx)) { ok = false; break; }
                    double* d = mxGetPr(fmx);
                    mwSize ne = mxGetNumberOfElements(fmx);
                    if (idx >= ne) { ok = false; break; }
                    vals[k] = d[idx];
                }
                if (ok) {
                    plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
                    double* out = mxGetPr(plhs[0]);
                    out[0]=vals[0]; out[1]=vals[1]; out[2]=vals[2];
                    return;
                }
            }
        }
    }
    mexErrMsgIdAndTxt("mex_matlab_helpers:field_not_found","None of the specified fields found in struct");
}

void do_has_field(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 3) mexErrMsgIdAndTxt("mex_matlab_helpers:has_field","Usage: mex_matlab_helpers('has_field', obs, fieldsCell)");
    const mxArray* obs = prhs[1];
    const mxArray* fields = prhs[2];
    mwSize n_fields = mxGetNumberOfElements(fields);
    for (mwIndex f = 0; f < n_fields; ++f) {
        std::string fname = getCellString(fields, f);
        if (fname.empty()) continue;
        if (mxGetField(obs, 0, fname.c_str()) != NULL) {
            plhs[0] = mxCreateLogicalScalar(true);
            return;
        }
    }
    plhs[0] = mxCreateLogicalScalar(false);
}

void do_get_euler(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 2) mexErrMsgIdAndTxt("mex_matlab_helpers:get_euler","Usage: mex_matlab_helpers('get_euler', quat)");
    const mxArray* qmx = prhs[1];
    if (!mxIsNumeric(qmx) || mxGetNumberOfElements(qmx) < 4) mexErrMsgIdAndTxt("mex_matlab_helpers:get_euler","quat must be numeric length 4");
    double* q = mxGetPr(qmx);
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double sinr_cosp = 2.0*(w*x + y*z);
    double cosr_cosp = 1.0 - 2.0*(x*x + y*y);
    double phi = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0*(w*y - z*x);
    double theta;
    if (fabs(sinp) >= 1.0) theta = copysign(M_PI/2.0, sinp); else theta = asin(sinp);

    double siny_cosp = 2.0*(w*z + x*y);
    double cosy_cosp = 1.0 - 2.0*(y*y + z*z);
    double psi = atan2(siny_cosp, cosy_cosp);

    plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
    double* out = mxGetPr(plhs[0]);
    out[0] = phi*180.0/M_PI; out[1] = theta*180.0/M_PI; out[2] = psi*180.0/M_PI;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs < 1) mexErrMsgIdAndTxt("mex_matlab_helpers:input","Command required");
    std::string cmd = getCommand(prhs[0]);
    if (cmd == "get_field") do_get_field(nlhs, plhs, nrhs, prhs);
    else if (cmd == "has_field") do_has_field(nlhs, plhs, nrhs, prhs);
    else if (cmd == "get_euler") do_get_euler(nlhs, plhs, nrhs, prhs);
    else mexErrMsgIdAndTxt("mex_matlab_helpers:unknown","Unknown command: %s", cmd.c_str());
}


