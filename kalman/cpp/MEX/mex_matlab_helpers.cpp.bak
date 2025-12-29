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
    const mxArray* idx_arr = prhs[3];
    double ncols_d = mxGetScalar(prhs[4]);
    int ncols = (int)ncols_d;
    
    // Check if idx is scalar or array
    bool is_scalar_idx = mxIsScalar(idx_arr);
    mwSize n_idx = 1;
    double* idx_data = NULL;
    std::vector<mwIndex> idx_vec;
    
    if (is_scalar_idx) {
        double idx_d = mxGetScalar(idx_arr);
        idx_vec.push_back((mwIndex)(idx_d - 1)); // MATLAB -> C
        n_idx = 1;
    } else {
        // Array indices
        if (!mxIsNumeric(idx_arr)) mexErrMsgIdAndTxt("mex_matlab_helpers:get_field","idx must be numeric");
        n_idx = mxGetNumberOfElements(idx_arr);
        idx_data = mxGetPr(idx_arr);
        idx_vec.resize(n_idx);
        for (mwSize i = 0; i < n_idx; ++i) {
            idx_vec[i] = (mwIndex)(idx_data[i] - 1); // MATLAB -> C
        }
    }

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
                    // Check all indices are valid
                    bool all_valid = true;
                    for (mwSize i = 0; i < n_idx; ++i) {
                        if (idx_vec[i] >= ne) { all_valid = false; break; }
                    }
                    if (all_valid) {
                        if (is_scalar_idx) {
                            plhs[0] = mxCreateDoubleScalar(data[idx_vec[0]]);
                        } else {
                            plhs[0] = mxCreateDoubleMatrix(n_idx, 1, mxREAL);
                            double* out = mxGetPr(plhs[0]);
                            for (mwSize i = 0; i < n_idx; ++i) {
                                out[i] = data[idx_vec[i]];
                            }
                        }
                        return;
                    }
                }
            } else if (ncols == 3) {
                // Check if field is a 3-column matrix
                if (mxIsNumeric(field) && mxGetNumberOfElements(field) >= 3) {
                    mwSize nrow = mxGetM(field);
                    mwSize ncol = mxGetN(field);
                    double* data = mxGetPr(field);
                    // Check all indices are valid
                    bool all_valid = true;
                    for (mwSize i = 0; i < n_idx; ++i) {
                        if (idx_vec[i] >= nrow) { all_valid = false; break; }
                    }
                    if (all_valid && ncol >= 3) {
                        if (is_scalar_idx) {
                            plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
                            double* out = mxGetPr(plhs[0]);
                            out[0] = data[idx_vec[0] + 0*nrow];
                            out[1] = data[idx_vec[0] + 1*nrow];
                            out[2] = data[idx_vec[0] + 2*nrow];
                        } else {
                            plhs[0] = mxCreateDoubleMatrix(n_idx, 3, mxREAL);
                            double* out = mxGetPr(plhs[0]);
                            for (mwSize i = 0; i < n_idx; ++i) {
                                out[i + 0*n_idx] = data[idx_vec[i] + 0*nrow];
                                out[i + 1*n_idx] = data[idx_vec[i] + 1*nrow];
                                out[i + 2*n_idx] = data[idx_vec[i] + 2*nrow];
                            }
                        }
                        return;
                    }
                }
                // Try to find x, y, z components
                std::string base = fname;
                // Remove "_x" suffix if present
                if (base.size() > 2 && base.substr(base.size()-2) == "_x") {
                    base = base.substr(0, base.size()-2);
                }
                
                // Special handling for single-character + 'x' pattern (like 'ax', 'wx', 'mx')
                // If fname is "ax", "wx", "mx", etc., extract the first character
                if (base.size() == 2 && base[1] == 'x') {
                    char base_char = base[0];
                    // Try direct pattern: ax, ay, az
                    std::string x_name = std::string(1, base_char) + "x";
                    std::string y_name = std::string(1, base_char) + "y";
                    std::string z_name = std::string(1, base_char) + "z";
                    
                    const mxArray* fx = mxGetField(obs, 0, x_name.c_str());
                    const mxArray* fy = mxGetField(obs, 0, y_name.c_str());
                    const mxArray* fz = mxGetField(obs, 0, z_name.c_str());
                    
                    if (fx && fy && fz && mxIsNumeric(fx) && mxIsNumeric(fy) && mxIsNumeric(fz)) {
                        double* dx = mxGetPr(fx);
                        double* dy = mxGetPr(fy);
                        double* dz = mxGetPr(fz);
                        mwSize nex = mxGetNumberOfElements(fx);
                        mwSize ney = mxGetNumberOfElements(fy);
                        mwSize nez = mxGetNumberOfElements(fz);
                        // Check all indices are valid
                        bool all_valid = true;
                        for (mwSize i = 0; i < n_idx; ++i) {
                            if (idx_vec[i] >= nex || idx_vec[i] >= ney || idx_vec[i] >= nez) {
                                all_valid = false; break;
                            }
                        }
                        if (all_valid) {
                            if (is_scalar_idx) {
                                plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
                                double* out = mxGetPr(plhs[0]);
                                out[0] = dx[idx_vec[0]];
                                out[1] = dy[idx_vec[0]];
                                out[2] = dz[idx_vec[0]];
                            } else {
                                plhs[0] = mxCreateDoubleMatrix(n_idx, 3, mxREAL);
                                double* out = mxGetPr(plhs[0]);
                                for (mwSize i = 0; i < n_idx; ++i) {
                                    out[i + 0*n_idx] = dx[idx_vec[i]];
                                    out[i + 1*n_idx] = dy[idx_vec[i]];
                                    out[i + 2*n_idx] = dz[idx_vec[i]];
                                }
                            }
                            return;
                        }
                    }
                }
                
                // Build candidate field names for other patterns
                std::vector<std::string> try_names;
                // Try standard format: base_x, base_y, base_z
                try_names.push_back(base + "_x");
                try_names.push_back(base + "_y");
                try_names.push_back(base + "_z");
                // Try short format: basex, basey, basez
                try_names.push_back(base + "x");
                try_names.push_back(base + "y");
                try_names.push_back(base + "z");
                
                // Special case: if base is a single character (like 'a', 'w', 'm'), try ay, az format
                if (base.size() == 1) {
                    char base_char = base[0];
                    try_names.push_back(std::string(1, base_char) + "y");
                    try_names.push_back(std::string(1, base_char) + "z");
                }
                
                // Try each candidate
                bool ok = true;
                double vals[3] = {0,0,0};
                int found_count = 0;
                
                // First, try to find x, y, z in order
                std::vector<const mxArray*> found_fields(3, NULL);
                for (int k=0; k<3 && found_count < 3; k++){
                    const char* nm = try_names[k].c_str();
                    const mxArray* fmx = mxGetField(obs, 0, nm);
                    if (fmx && mxIsNumeric(fmx)) {
                        double* d = mxGetPr(fmx);
                        mwSize ne = mxGetNumberOfElements(fmx);
                        // Check all indices are valid
                        bool all_valid = true;
                        for (mwSize i = 0; i < n_idx; ++i) {
                            if (idx_vec[i] >= ne) { all_valid = false; break; }
                        }
                        if (all_valid) {
                            found_fields[found_count] = fmx;
                            found_count++;
                        }
                    }
                }
                
                // If we found 3 components, return them
                if (found_count == 3) {
                    if (is_scalar_idx) {
                        plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);
                        double* out = mxGetPr(plhs[0]);
                        out[0] = mxGetPr(found_fields[0])[idx_vec[0]];
                        out[1] = mxGetPr(found_fields[1])[idx_vec[0]];
                        out[2] = mxGetPr(found_fields[2])[idx_vec[0]];
                    } else {
                        plhs[0] = mxCreateDoubleMatrix(n_idx, 3, mxREAL);
                        double* out = mxGetPr(plhs[0]);
                        for (mwSize i = 0; i < n_idx; ++i) {
                            out[i + 0*n_idx] = mxGetPr(found_fields[0])[idx_vec[i]];
                            out[i + 1*n_idx] = mxGetPr(found_fields[1])[idx_vec[i]];
                            out[i + 2*n_idx] = mxGetPr(found_fields[2])[idx_vec[i]];
                        }
                    }
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


