function remove_japanese_comments()
    % Remove Japanese comments from MEX header files to fix C4819 encoding warnings
    
    mex_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'Inc', 'MEX');
    files = {
        'mex_eskf_common.hpp'
        'mex_run_eskf_filter_ops.hpp'
        'mex_run_eskf_impl.hpp'
        'mex_run_eskf_sensor_updates.hpp'
        'mex_helpers.hpp'
    };
    
    for i = 1:length(files)
        fname = fullfile(mex_dir, files{i});
        if isfile(fname)
            fix_file(fname);
        end
    end
end

function fix_file(fname)
    % Read the entire file as UTF-8
    try
        fid = fopen(fname, 'r', 'n', 'UTF-8');
        content = fread(fid, '*char')';
        fclose(fid);
    catch ME
        fprintf('Error reading %s: %s\n', fname, ME.message);
        return;
    end
    
    % Replace Japanese comments with English equivalents
    replacements = {
        % mex_eskf_common.hpp
        'mex_run_eskf.cpp用の共通インクルードと定義' , 'Common includes and definitions for mex_run_eskf.cpp'
        'このヘッダーは、ESKF関連のMEXファイルで共通して使用される' , 'This header contains common includes, definitions, and'
        'インクルード、定義、using宣言をまとめています。' , 'using declarations used in ESKF-related MEX files.'
        '標準ライブラリ' , 'Standard library'
        '定数定義' , 'Constant definitions'
        'レイヤー1: 基本型（最初に配置）' , 'Layer 1: Basic types (placed first)'
        'レイヤー2: ユーティリティ' , 'Layer 2: Utilities'
        'レイヤー3: ESKF コア' , 'Layer 3: ESKF core'
        'レイヤー4: 統合層' , 'Layer 4: Integration layer'
        'using宣言' , 'Using declarations'
        'マクロ定義（センサーデータ取得用）' , 'Macro definitions (for sensor data retrieval)'
        'Note: マクロ内では名前空間解決が効かないため、完全修飾名を使用' , 'Note: Namespace resolution does not work inside macros, use fully qualified names'
        'また、Unicodeエンコーディングの問題を避けるため、コメントは最小限に' , 'Keep comments minimal to avoid Unicode encoding issues'
        'グローバル変数の前方宣言（mex_run_eskf_impl名前空間用）' , 'Forward declaration of global variables (for mex_run_eskf_impl namespace)'
        
        % mex_run_eskf_filter_ops.hpp
        'mex_run_eskf.cpp用のフィルター操作関数群' , 'Filter operation functions for mex_run_eskf.cpp'
        'リセットチェック、ZUPTチェックなどの実装を含みます。' , 'Contains implementations of reset checks, ZUPT checks, etc.'
        'リセットチェックと処理' , 'Reset check and processing'
        'ZUPTチェックと更新処理' , 'ZUPT check and update processing'
        
        % mex_run_eskf_impl.hpp
        'mex_run_eskf.cpp用の実装関数群' , 'Implementation functions for mex_run_eskf.cpp'
        'このヘッダーには、mex_run_eskf.cppで使用される内部関数の定義が含まれます。' , 'This header contains definitions of internal functions used in mex_run_eskf.cpp.'
        'グローバル変数とstatic関数をinline化してヘッダーに移動しています。' , 'Global variables and static functions have been inlined and moved to header.'
        'グローバル変数（extern宣言、実装は.cppファイルに）' , 'Global variables (extern declarations, implementations in .cpp file)'
        '予測ステップの呼び出し（ESKFRunnerを使用）' , 'Predict step call (using ESKFRunner)'
        '初期化処理' , 'Initialization'
        'ステップ処理' , 'Step processing'
        '状態取得処理' , 'Get state'
        'メモリ解放処理' , 'Free memory'
    };
    
    for i = 1:2:length(replacements)
        from_str = replacements{i};
        to_str = replacements{i+1};
        content = strrep(content, from_str, to_str);
    end
    
    % Write back
    try
        fid = fopen(fname, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', content);
        fclose(fid);
        fprintf('Fixed: %s\n', fname);
    catch ME
        fprintf('Error writing %s: %s\n', fname, ME.message);
    end
end
