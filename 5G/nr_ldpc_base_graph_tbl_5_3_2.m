%[i,j,V_i_j] = nr_ldpc_base_graph_tbl_5_3_2(base_graph, i_LS)
%
% Returns parameters of a given LDPC base graph and parity
% check matrix.
% Implements tables from 3GPP 38.212 5.3.2-2 and 5.3.2-3.
%
% Arguments:
%  base_graph - LDPC base graph (1 or 2) 
%  i_LS       - lifting size set index
%
% Returns:
%  i          - base graph row index
%  j          - base graph column index
%  V_i_j      - cyclic shift per row i and column j

% Copyright 2018 Grzegorz Cisek (grzegorzcisek@gmail.com)

function [i,j,V_i_j] = nr_ldpc_base_graph_tbl_5_3_2(base_graph, i_LS)

persistent i_bg1 j_bg1 V_i_j_bg1 i_bg2 j_bg2 V_i_j_bg2
 
if isempty(i_bg1)
  i_bg1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 27, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 36, 36, 36, 36, 36, 37, 37, 37, 37, 38, 38, 38, 38, 38, 39, 39, 39, 39, 39, 40, 40, 40, 40, 41, 41, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 45, 45, 45, 45];
end
if isempty(j_bg1)
  j_bg1 = [0, 1, 2, 3, 5, 6, 9, 10, 11, 12, 13, 15, 16, 18, 19, 20, 21, 22, 23, 0, 2, 3, 4, 5, 7, 8, 9, 11, 12, 14, 15, 16, 17, 19, 21, 22, 23, 24, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15, 17, 18, 19, 20, 24, 25, 0, 1, 3, 4, 6, 7, 8, 10, 11, 12, 13, 14, 16, 17, 18, 20, 21, 22, 25, 0, 1, 26, 0, 1, 3, 12, 16, 21, 22, 27, 0, 6, 10, 11, 13, 17, 18, 20, 28, 0, 1, 4, 7, 8, 14, 29, 0, 1, 3, 12, 16, 19, 21, 22, 24, 30, 0, 1, 10, 11, 13, 17, 18, 20, 31, 1, 2, 4, 7, 8, 14, 32, 0, 1, 12, 16, 21, 22, 23, 33, 0, 1, 10, 11, 13, 18, 34, 0, 3, 7, 20, 23, 35, 0, 12, 15, 16, 17, 21, 36, 0, 1, 10, 13, 18, 25, 37, 1, 3, 11, 20, 22, 38, 0, 14, 16, 17, 21, 39, 1, 12, 13, 18, 19, 40, 0, 1, 7, 8, 10, 41, 0, 3, 9, 11, 22, 42, 1, 5, 16, 20, 21, 43, 0, 12, 13, 17, 44, 1, 2, 10, 18, 45, 0, 3, 4, 11, 22, 46, 1, 6, 7, 14, 47, 0, 2, 4, 15, 48, 1, 6, 8, 49, 0, 4, 19, 21, 50, 1, 14, 18, 25, 51, 0, 10, 13, 24, 52, 1, 7, 22, 25, 53, 0, 12, 14, 24, 54, 1, 2, 11, 21, 55, 0, 7, 15, 17, 56, 1, 6, 12, 22, 57, 0, 14, 15, 18, 58, 1, 13, 23, 59, 0, 9, 10, 12, 60, 1, 3, 7, 19, 61, 0, 8, 17, 62, 1, 3, 9, 18, 63, 0, 4, 24, 64, 1, 16, 18, 25, 65, 0, 7, 9, 22, 66, 1, 6, 10, 67];
end
if isempty(V_i_j_bg1)
  V_i_j_bg1 = [250  307  73   223  211  294  0    135
               69   19   15   16   198  118  0    227
               226  50   103  94   188  167  0    126
               159  369  49   91   186  330  0    134
               100  181  240  74   219  207  0    84
               10   216  39   10   4    165  0    83
               59   317  15   0    29   243  0    53
               229  288  162  205  144  250  0    225
               110  109  215  216  116  1    0    205
               191  17   164  21   216  339  0    128
               9    357  133  215  115  201  0    75
               195  215  298  14   233  53   0    135
               23   106  110  70   144  347  0    217
               190  242  113  141  95   304  0    220
               35   180  16   198  216  167  0    90
               239  330  189  104  73   47   0    105
               31   346  32   81   261  188  0    137
               1    1    1    1    1    1    0    1
               0    0    0    0    0    0    0    0
               2    76   303  141  179  77   22   96
               239  76   294  45   162  225  11   236
               117  73   27   151  223  96   124  136
               124  288  261  46   256  338  0    221
               71   144  161  119  160  268  10   128
               222  331  133  157  76   112  0    92
               104  331  4    133  202  302  0    172
               173  178  80   87   117  50   2    56
               220  295  129  206  109  167  16   11
               102  342  300  93   15   253  60   189
               109  217  76   79   72   334  0    95
               132  99   266  9    152  242  6    85
               142  354  72   118  158  257  30   153
               155  114  83   194  147  133  0    87
               255  331  260  31   156  9    168  163
               28   112  301  187  119  302  31   216
               0    0    0    0    0    0    105  0
               0    0    0    0    0    0    0    0
               0    0    0    0    0    0    0    0
               106  205  68   207  258  226  132  189
               111  250  7    203  167  35   37   4
               185  328  80   31   220  213  21   225
               63   332  280  176  133  302  180  151
               117  256  38   180  243  111  4    236
               93   161  227  186  202  265  149  117
               229  267  202  95   218  128  48   179
               177  160  200  153  63   237  38   92
               95   63   71   177  0    294  122  24
               39   129  106  70   3    127  195  68
               142  200  295  77   74   110  155  6
               225  88   283  214  229  286  28   101
               225  53   301  77   0    125  85   33
               245  131  184  198  216  131  47   96
               205  240  246  117  269  163  179  125
               251  205  230  223  200  210  42   67
               117  13   276  90   234  7    66   230
               0    0    0    0    0    0    0    0
               0    0    0    0    0    0    0    0
               121  276  220  201  187  97   4    128
               89   87   208  18   145  94   6    23
               84   0    30   165  166  49   33   162
               20   275  197  5    108  279  113  220
               150  199  61   45   82   139  49   43
               131  153  175  142  132  166  21   186
               243  56   79   16   197  91   6    96
               136  132  281  34   41   106  151  1
               86   305  303  155  162  246  83   216
               246  231  253  213  57   345  154  22
               219  341  164  147  36   269  87   24
               211  212  53   69   115  185  5    167
               240  304  44   96   242  249  92   200
               76   300  28   74   165  215  173  32
               244  271  77   99   0    143  120  235
               144  39   319  30   113  121  2    172
               12   357  68   158  108  121  142  219
               1    1    1    1    1    1    0    1
               0    0    0    0    0    0    0    0
               157  332  233  170  246  42   24   64
               102  181  205  10   235  256  204  211
               0    0    0    0    0    0    0    0
               205  195  83   164  261  219  185  2
               236  14   292  59   181  130  100  171
               194  115  50   86   72   251  24   47
               231  166  318  80   283  322  65   143
               28   241  201  182  254  295  207  210
               123  51   267  130  79   258  161  180
               115  157  279  153  144  283  72   180
               0    0    0    0    0    0    0    0
               183  278  289  158  80   294  6    199
               22   257  21   119  144  73   27   22
               28   1    293  113  169  330  163  23
               67   351  13   21   90   99   50   100
               244  92   232  63   59   172  48   92
               11   253  302  51   177  150  24   207
               157  18   138  136  151  284  38   52
               211  225  235  116  108  305  91   13
               0    0    0    0    0    0    0    0
               220  9    12   17   169  3    145  77
               44   62   88   76   189  103  88   146
               159  316  207  104  154  224  112  209
               31   333  50   100  184  297  153  32
               167  290  25   150  104  215  159  166
               104  114  76   158  164  39   76   18
               0    0    0    0    0    0    0    0
               112  307  295  33   54   348  172  181
               4    179  133  95   0    75   2    105
               7    165  130  4    252  22   131  141
               211  18   231  217  41   312  141  223
               102  39   296  204  98   224  96   177
               164  224  110  39   46   17   99   145
               109  368  269  58   15   59   101  199
               241  67   245  44   230  314  35   153
               90   170  154  201  54   244  116  38
               0    0    0    0    0    0    0    0
               103  366  189  9    162  156  6    169
               182  232  244  37   159  88   10   12
               109  321  36   213  93   293  145  206
               21   133  286  105  134  111  53   221
               142  57   151  89   45   92   201  17
               14   303  267  185  132  152  4    212
               61   63   135  109  76   23   164  92
               216  82   209  218  209  337  173  205
               0    0    0    0    0    0    0    0
               98   101  14   82   178  175  126  116
               149  339  80   165  1    253  77   151
               167  274  211  174  28   27   156  70
               160  111  75   19   267  231  16   230
               49   383  161  194  234  49   12   115
               58   354  311  103  201  267  70   84
               0    0    0    0    0    0    0    0
               77   48   16   52   55   25   184  45
               41   102  147  11   23   322  194  115
               83   8    290  2    274  200  123  134
               182  47   289  35   181  351  16   1
               78   188  177  32   273  166  104  152
               252  334  43   84   39   338  109  165
               22   115  280  201  26   192  124  107
               0    0    0    0    0    0    0    0
               160  77   229  142  225  123  6    186
               42   186  235  175  162  217  20   215
               21   174  169  136  244  142  203  124
               32   232  48   3    151  110  153  180
               234  50   105  28   238  176  104  98
               7    74   52   182  243  76   207  80
               0    0    0    0    0    0    0    0
               177  313  39   81   231  311  52   220
               248  177  302  56   0    251  147  185
               151  266  303  72   216  265  1    154
               185  115  160  217  47   94   16   178
               62   370  37   78   36   81   46   150
               0    0    0    0    0    0    0    0
               206  142  78   14   0    22   1    124
               55   248  299  175  186  322  202  144
               206  137  54   211  253  277  118  182
               127  89   61   191  16   156  130  95
               16   347  179  51   0    66   1    72
               229  12   258  43   79   78   2    76
               0    0    0    0    0    0    0    0
               40   241  229  90   170  176  173  39
               96   2    290  120  0    348  6    138
               65   210  60   131  183  15   81   220
               63   318  130  209  108  81   182  173
               75   55   184  209  68   176  53   142
               179  269  51   81   64   113  46   49
               0    0    0    0    0    0    0    0
               64   13   69   154  270  190  88   78
               49   338  140  164  13   293  198  152
               49   57   45   43   99   332  160  84
               51   289  115  189  54   331  122  5
               154  57   300  101  0    114  182  205
               0    0    0    0    0    0    0    0
               7    260  257  56   153  110  91   183
               164  303  147  110  137  228  184  112
               59   81   128  200  0    247  30   106
               1    358  51   63   0    116  3    219
               144  375  228  4    162  190  155  129
               0    0    0    0    0    0    0    0
               42   130  260  199  161  47   1    183
               233  163  294  110  151  286  41   215
               8    280  291  200  0    246  167  180
               155  132  141  143  241  181  68   143
               147  4    295  186  144  73   148  14
               0    0    0    0    0    0    0    0
               60   145  64   8    0    87   12   179
               73   213  181  6    0    110  6    108
               72   344  101  103  118  147  166  159
               127  242  270  198  144  258  184  138
               224  197  41   8    0    204  191  196
               0    0    0    0    0    0    0    0
               151  187  301  105  265  89   6    77
               186  206  162  210  81   65   12   187
               217  264  40   121  90   155  15   203
               47   341  130  214  144  244  5    167
               160  59   10   183  228  30   30   130
               0    0    0    0    0    0    0    0
               249  205  79   192  64   162  6    197
               121  102  175  131  46   264  86   122
               109  328  132  220  266  346  96   215
               131  213  283  50   9    143  42   65
               171  97   103  106  18   109  199  216
               0    0    0    0    0    0    0    0
               64   30   177  53   72   280  44   25
               142  11   20   0    189  157  58   47
               188  233  55   3    72   236  130  126
               158  22   316  148  257  113  131  178
               0    0    0    0    0    0    0    0
               156  24   249  88   180  18   45   185
               147  89   50   203  0    6    18   127
               170  61   133  168  0    181  132  117
               152  27   105  122  165  304  100  199
               0    0    0    0    0    0    0    0
               112  298  289  49   236  38   9    32
               86   158  280  157  199  170  125  178
               236  235  110  64   0    249  191  2
               116  339  187  193  266  288  28   156
               222  234  281  124  0    194  6    58
               0    0    0    0    0    0    0    0
               23   72   172  1    205  279  4    27
               136  17   295  166  0    255  74   141
               116  383  96   65   0    111  16   11
               182  312  46   81   183  54   28   181
               0    0    0    0    0    0    0    0
               195  71   270  107  0    325  21   163
               243  81   110  176  0    326  142  131
               215  76   318  212  0    226  192  169
               61   136  67   127  277  99   197  98
               0    0    0    0    0    0    0    0
               25   194  210  208  45   91   98   165
               104  194  29   141  36   326  140  232
               194  101  304  174  72   268  22   9
               0    0    0    0    0    0    0    0
               128  222  11   146  275  102  4    32
               165  19   293  153  0    1    1    43
               181  244  50   217  155  40   40   200
               63   274  234  114  62   167  93   205
               0    0    0    0    0    0    0    0
               86   252  27   150  0    273  92   232
               236  5    308  11   180  104  136  32
               84   147  117  53   0    243  106  118
               6    78   29   68   42   107  6    103
               0    0    0    0    0    0    0    0
               216  159  91   34   0    171  2    170
               73   229  23   130  90   16   88   199
               120  260  105  210  252  95   112  26
               9    90   135  123  173  212  20   105
               0    0    0    0    0    0    0    0
               95   100  222  175  144  101  4    73
               177  215  308  49   144  297  49   149
               172  258  66   177  166  279  125  175
               61   256  162  128  19   222  194  108
               0    0    0    0    0    0    0    0
               221  102  210  192  0    351  6    103
               112  201  22   209  211  265  126  110
               199  175  271  58   36   338  63   151
               121  287  217  30   162  83   20   211
               0    0    0    0    0    0    0    0
               2    323  170  114  0    56   10   199
               187  8    20   49   0    304  30   132
               41   361  140  161  76   141  6    172
               211  105  33   137  18   101  92   65
               0    0    0    0    0    0    0    0
               127  230  187  82   197  60   4    161
               167  148  296  186  0    320  153  237
               164  202  5    68   108  112  197  142
               159  312  44   150  0    54   155  180
               0    0    0    0    0    0    0    0
               161  320  207  192  199  100  4    231
               197  335  158  173  278  210  45   174
               207  2    55   26   0    195  168  145
               103  266  285  187  205  268  185  100
               0    0    0    0    0    0    0    0
               37   210  259  222  216  135  6    11
               105  313  179  157  16   15   200  207
               51   297  178  0    0    35   177  42
               120  21   160  6    0    188  43   100
               0    0    0    0    0    0    0    0
               198  269  298  81   72   319  82   59
               220  82   15   195  144  236  2    204
               122  115  115  138  0    85   135  161
               0    0    0    0    0    0    0    0
               167  185  151  123  190  164  91   121
               151  177  179  90   0    196  64   90
               157  289  64   73   0    209  198  26
               163  214  181  10   0    246  100  140
               0    0    0    0    0    0    0    0
               173  258  102  12   153  236  4    115
               139  93   77   77   0    264  28   188
               149  346  192  49   165  37   109  168
               0    297  208  114  117  272  188  52
               0    0    0    0    0    0    0    0
               157  175  32   67   216  304  10   4
               137  37   80   45   144  237  84   103
               149  312  197  96   2    135  12   30
               0    0    0    0    0    0    0    0
               167  52   154  23   0    123  2    53
               173  314  47   215  0    77   75   189
               139  139  124  60   0    25   142  215
               151  288  207  167  183  272  128  24
               0    0    0    0    0    0    0    0
               149  113  226  114  27   288  163  222
               157  14   65   91   0    83   10   170
               137  218  126  78   35   17   162  71
               0    0    0    0    0    0    0    0
               151  113  228  206  52   210  1    22
               163  132  69   22   243  3    163  127
               173  114  176  134  0    53   99   49
               139  168  102  161  270  167  98   125
               0    0    0    0    0    0    0    0
               139  80   234  84   18   79   4    191
               157  78   227  4    0    244  6    211
               163  163  259  9    0    293  142  187
               173  274  260  12   57   272  3    148
               0    0    0    0    0    0    0    0
               149  135  101  184  168  82   181  177
               151  149  228  121  0    67   45   114
               167  15   126  29   144  235  153  93
               0    0    0    0    0    0    0    0];
end
if isempty(i_bg2)
  i_bg2 = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 23, 23, 23, 23, 24, 24, 24, 24, 25, 25, 25, 26, 26, 26, 26, 26, 27, 27, 27, 28, 28, 28, 28, 29, 29, 29, 30, 30, 30, 30, 30, 31, 31, 31, 32, 32, 32, 32, 33, 33, 33, 33, 34, 34, 34, 34, 35, 35, 35, 35, 36, 36, 36, 36, 37, 37, 37, 38, 38, 38, 38, 39, 39, 39, 39, 40, 40, 40, 40, 41, 41, 41, 41];
end
if isempty(j_bg2)
  j_bg2 = [0, 1, 2, 3, 6, 9, 10, 11, 0, 3, 4, 5, 6, 7, 8, 9, 11, 12, 0, 1, 3, 4, 8, 10, 12, 13, 1, 2, 4, 5, 6, 7, 8, 9, 10, 13, 0, 1, 11, 14, 0, 1, 5, 7, 11, 15, 0, 5, 7, 9, 11, 16, 1, 5, 7, 11, 13, 17, 0, 1, 12, 18, 1, 8, 10, 11, 19, 0, 1, 6, 7, 20, 0, 7, 9, 13, 21, 1, 3, 11, 22, 0, 1, 8, 13, 23, 1, 6, 11, 13, 24, 0, 10, 11, 25, 1, 9, 11, 12, 26, 1, 5, 11, 12, 27, 0, 6, 7, 28, 0, 1, 10, 29, 1, 4, 11, 30, 0, 8, 13, 31, 1, 2, 32, 0, 3, 5, 33, 1, 2, 9, 34, 0, 5, 35, 2, 7, 12, 13, 36, 0, 6, 37, 1, 2, 5, 38, 0, 4, 39, 2, 5, 7, 9, 40, 1, 13, 41, 0, 5, 12, 42, 2, 7, 10, 43, 0, 12, 13, 44, 1, 5, 11, 45, 0, 2, 7, 46, 10, 13, 47, 1, 5, 11, 48, 0, 7, 12, 49, 2, 10, 13, 50, 1, 5, 11, 51];
end
if isempty(V_i_j_bg2)
  V_i_j_bg2 = [9    174  0    72   3    156  143  145
               117  97   0    110  26   143  19   131
               204  166  0    23   53   14   176  71
               26   66   0    181  35   3    165  21
               189  71   0    95   115  40   196  23
               205  172  0    8    127  123  13   112
               0    0    0    1    0    0    0    1
               0    0    0    0    0    0    0    0
               167  27   137  53   19   17   18   142
               166  36   124  156  94   65   27   174
               253  48   0    115  104  63   3    183
               125  92   0    156  66   1    102  27
               226  31   88   115  84   55   185  96
               156  187  0    200  98   37   17   23
               224  185  0    29   69   171  14   9
               252  3    55   31   50   133  180  167
               0    0    0    0    0    0    0    0
               0    0    0    0    0    0    0    0
               81   25   20   152  95   98   126  74
               114  114  94   131  106  168  163  31
               44   117  99   46   92   107  47   3
               52   110  9    191  110  82   183  53
               240  114  108  91   111  142  132  155
               1    1    1    0    1    1    1    0
               0    0    0    0    0    0    0    0
               0    0    0    0    0    0    0    0
               8    136  38   185  120  53   36   239
               58   175  15   6    121  174  48   171
               158  113  102  36   22   174  18   95
               104  72   146  124  4    127  111  110
               209  123  12   124  73   17   203  159
               54   118  57   110  49   89   3    199
               18   28   53   156  128  17   191  43
               128  186  46   133  79   105  160  75
               0    0    0    1    0    0    0    1
               0    0    0    0    0    0    0    0
               179  72   0    200  42   86   43   29
               214  74   136  16   24   67   27   140
               71   29   157  101  51   83   117  180
               0    0    0    0    0    0    0    0
               231  10   0    185  40   79   136  121
               41   44   131  138  140  84   49   41
               194  121  142  170  84   35   36   169
               159  80   141  219  137  103  132  88
               103  48   64   193  71   60   62   207
               0    0    0    0    0    0    0    0
               155  129  0    123  109  47   7    137
               228  92   124  55   87   154  34   72
               45   100  99   31   107  10   198  172
               28   49   45   222  133  155  168  124
               158  184  148  209  139  29   12   56
               0    0    0    0    0    0    0    0
               129  80   0    103  97   48   163  86
               147  186  45   13   135  125  78   186
               140  16   148  105  35   24   143  87
               3    102  96   150  108  47   107  172
               116  143  78   181  65   55   58   154
               0    0    0    0    0    0    0    0
               142  118  0    147  70   53   101  176
               94   70   65   43   69   31   177  169
               230  152  87   152  88   161  22   225
               0    0    0    0    0    0    0    0
               203  28   0    2    97   104  186  167
               205  132  97   30   40   142  27   238
               61   185  51   184  24   99   205  48
               247  178  85   83   49   64   81   68
               0    0    0    0    0    0    0    0
               11   59   0    174  46   111  125  38
               185  104  17   150  41   25   60   217
               0    22   156  8    101  174  177  208
               117  52   20   56   96   23   51   232
               0    0    0    0    0    0    0    0
               11   32   0    99   28   91   39   178
               236  92   7    138  30   175  29   214
               210  174  4    110  116  24   35   168
               56   154  2    99   64   141  8    51
               0    0    0    0    0    0    0    0
               63   39   0    46   33   122  18   124
               111  93   113  217  122  11   155  122
               14   11   48   109  131  4    49   72
               0    0    0    0    0    0    0    0
               83   49   0    37   76   29   32   48
               2    125  112  113  37   91   53   57
               38   35   102  143  62   27   95   167
               222  166  26   140  47   127  186  219
               0    0    0    0    0    0    0    0
               115  19   0    36   143  11   91   82
               145  118  138  95   51   145  20   232
               3    21   57   40   130  8    52   204
               232  163  27   116  97   166  109  162
               0    0    0    0    0    0    0    0
               51   68   0    116  139  137  174  38
               175  63   73   200  96   103  108  217
               213  81   99   110  128  40   102  157
               0    0    0    0    0    0    0    0
               203  87   0    75   48   78   125  170
               142  177  79   158  9    158  31   23
               8    135  111  134  28   17   54   175
               242  64   143  97   8    165  176  202
               0    0    0    0    0    0    0    0
               254  158  0    48   120  134  57   196
               124  23   24   132  43   23   201  173
               114  9    109  206  65   62   142  195
               64   6    18   2    42   163  35   218
               0    0    0    0    0    0    0    0
               220  186  0    68   17   173  129  128
               194  6    18   16   106  31   203  211
               50   46   86   156  142  22   140  210
               0    0    0    0    0    0    0    0
               87   58   0    35   79   13   110  39
               20   42   158  138  28   135  124  84
               185  156  154  86   41   145  52   88
               0    0    0    0    0    0    0    0
               26   76   0    6    2    128  196  117
               105  61   148  20   103  52   35   227
               29   153  104  141  78   173  114  6
               0    0    0    0    0    0    0    0
               76   157  0    80   91   156  10   238
               42   175  17   43   75   166  122  13
               210  67   33   81   81   40   23   11
               0    0    0    0    0    0    0    0
               222  20   0    49   54   18   202  195
               63   52   4    1    132  163  126  44
               0    0    0    0    0    0    0    0
               23   106  0    156  68   110  52   5
               235  86   75   54   115  132  170  94
               238  95   158  134  56   150  13   111
               0    0    0    0    0    0    0    0
               46   182  0    153  30   113  113  81
               139  153  69   88   42   108  161  19
               8    64   87   63   101  61   88   130
               0    0    0    0    0    0    0    0
               228  45   0    211  128  72   197  66
               156  21   65   94   63   136  194  95
               0    0    0    0    0    0    0    0
               29   67   0    90   142  36   164  146
               143  137  100  6    28   38   172  66
               160  55   13   221  100  53   49   190
               122  85   7    6    133  145  161  86
               0    0    0    0    0    0    0    0
               8    103  0    27   13   42   168  64
               151  50   32   118  10   104  193  181
               0    0    0    0    0    0    0    0
               98   70   0    216  106  64   14   7
               101  111  126  212  77   24   186  144
               135  168  110  193  43   149  46   16
               0    0    0    0    0    0    0    0
               18   110  0    108  133  139  50   25
               28   17   154  61   25   161  27   57
               0    0    0    0    0    0    0    0
               71   120  0    106  87   84   70   37
               240  154  35   44   56   173  17   139
               9    52   51   185  104  93   50   221
               84   56   134  176  70   29   6    17
               0    0    0    0    0    0    0    0
               106  3    0    147  80   117  115  201
               1    170  20   182  139  148  189  46
               0    0    0    0    0    0    0    0
               242  84   0    108  32   116  110  179
               44   8    20   21   89   73   0    14
               166  17   122  110  71   142  163  116
               0    0    0    0    0    0    0    0
               132  165  0    71   135  105  163  46
               164  179  88   12   6    137  173  2
               235  124  13   109  2    29   179  106
               0    0    0    0    0    0    0    0
               147  173  0    29   37   11   197  184
               85   177  19   201  25   41   191  135
               36   12   78   69   114  162  193  141
               0    0    0    0    0    0    0    0
               57   77   0    91   60   126  157  85
               40   184  157  165  137  152  167  225
               63   18   6    55   93   172  181  175
               0    0    0    0    0    0    0    0
               140  25   0    1    121  73   197  178
               38   151  63   175  129  154  167  112
               154  170  82   83   26   129  179  106
               0    0    0    0    0    0    0    0
               219  37   0    40   97   167  181  154
               151  31   144  12   56   38   193  114
               0    0    0    0    0    0    0    0
               31   84   0    37   1    112  157  42
               66   151  93   97   70   7    173  41
               38   190  19   46   1    19   191  105
               0    0    0    0    0    0    0    0
               239  93   0    106  119  109  181  167
               172  132  24   181  32   6    157  45
               34   57   138  154  142  105  173  189
               0    0    0    0    0    0    0    0
               0    103  0    98   6    160  193  78
               75   107  36   35   73   156  163  67
               120  163  143  36   102  82   179  180
               0    0    0    0    0    0    0    0
               129  147  0    120  48   132  191  53
               229  7    2    101  47   6    197  215
               118  60   55   81   19   8    167  230
               0    0    0    0    0    0    0    0];
end

if base_graph == 1
  i = i_bg1;
  j = j_bg1;
  V_i_j = V_i_j_bg1(:,i_LS+1);
elseif base_graph == 2
  i = i_bg2;
  j = j_bg2;
  V_i_j = V_i_j_bg2(:,i_LS+1);
else
  error('base_graph permitted values are 1 or 2');
end

end