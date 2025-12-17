function parse_debug_mex(infile,outfile)
% parse_debug_mex  Load a debug .mat and write readable summary to text file
% Usage: parse_debug_mex('path/to/debug.mat','path/to/out.txt')

dump = load(infile);
fn = fieldnames(dump);
fid = fopen(outfile,'w');
if fid==-1
    error('Cannot open output file: %s', outfile);
end
fprintf(fid,'Parsed file: %s\n\n', infile);
for i=1:numel(fn)
    name = fn{i};
    v = dump.(name);
    fprintf(fid,'--- %s ---\n', name);
    try
        if isnumeric(v)
            s = size(v);
            if numel(v) <= 100
                fprintf(fid,'size: %s\n', mat2str(s));
                fprintf(fid,'%s\n', mat2str(v(:)'));
            else
                fprintf(fid,'size: %s (first 20 elements)\n', mat2str(s));
                idx = min(20, numel(v));
                fprintf(fid,'%s\n', mat2str(v(1:idx)'));
            end
        elseif ischar(v) || isstring(v)
            fprintf(fid,'type: %s\n', class(v));
            fprintf(fid,'%s\n', char(v));
        elseif isstruct(v)
            sub = fieldnames(v);
            fprintf(fid,'struct with fields: %s\n', strjoin(sub,', '));
        else
            fprintf(fid,'type: %s\n', class(v));
        end
    catch ME
        fprintf(fid,'(error formatting variable: %s)\n', ME.message);
    end
    fprintf(fid,'\n');
end
fclose(fid);
fprintf('Wrote parsed debug to %s\n', outfile);
end
