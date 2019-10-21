% Author: Dongsheng Deng
% Homepage: https://ddswhu.me/
% Email: elegantlatex2e@gmail.com
% Lastest Version: https://github.com/ElegantLaTeX/ElegantPaper

% --- Class structure: identification part
\ProvidesClass{elegantpaper}[2019/04/01 v0.07 ElegantLaTeX Paper class]
\NeedsTeXFormat{LaTeX2e} 

\RequirePackage{kvoptions}
\RequirePackage{ifthen}
\RequirePackage{ifxetex}
\RequirePackage{etoolbox}
\RequirePackage{calc}

\SetupKeyvalOptions{family=ELEGANT, prefix=ELEGANT@, setkeys=\kvsetkeys}
\newcommand{\ekv}[1]{\kvsetkeys{ELEGANT}{#1}}

\DeclareStringOption[en]{lang}
\DeclareVoidOption{cn}{\ekv{lang = cn}}
\DeclareVoidOption{en}{\ekv{lang = en}}

\DeclareStringOption[numbers]{cite}[numbers]
\DeclareVoidOption{numbers}{\ekv{cite=numbers}}
\DeclareVoidOption{authoryear}{\ekv{cite=authoryear}}
\DeclareVoidOption{super}{\ekv{cite=super}}
% \PassOptionsToClass{11pt}{article}

\DeclareDefaultOption{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessKeyvalOptions*\relax
\LoadClass{article}



\RequirePackage{amsthm}
\RequirePackage{amsmath,amssymb}

\linespread{1.35}


% indentation, table and figure package
\RequirePackage{indentfirst}
\RequirePackage{booktabs}
\RequirePackage{multicol}
\RequirePackage{multirow,linegoal}
\RequirePackage{xcolor}
\RequirePackage{graphicx}
\RequirePackage{fancyvrb}
\RequirePackage{abstract}

\graphicspath{{image/}{figure/}{fig/}{img/}}

% caption settings 
\RequirePackage[font=small,labelfont={bf}]{caption} 
\captionsetup[table]{skip=3pt}
\captionsetup[figure]{skip=3pt}

% list/itemize/enumerate setting
\RequirePackage[shortlabels,inline]{enumitem}
\setlist{nolistsep}


% define the hyperref color 
\RequirePackage{xcolor}
\definecolor{winered}{rgb}{0.5,0,0}

% bib-settings
\ifdefstring{\ELEGANT@cite}{authoryear}{
	\RequirePackage[authoryear,sort&compress]{natbib}
}{\relax}
\ifdefstring{\ELEGANT@cite}{numbers}{
	\RequirePackage[numbers,sort&compress]{natbib}
}{\relax}
\ifdefstring{\ELEGANT@cite}{super}{
	\RequirePackage[super,sort&compress]{natbib}
}{\relax}

\setlength{\bibsep}{0.0pt}
\def\bibfont{\footnotesize}

% settings for the geometry
\RequirePackage{hyperref}


\hypersetup{
  pdfborder={0 0 0},
  colorlinks=true,
  linkcolor={winered},
  urlcolor={winered},
  filecolor={winered},
  citecolor={winered},
  linktoc=all,
}
\RequirePackage[left=1in, right=1in, top=1in, bottom=1in,headheight=0pt,headsep=0pt]{geometry}

% \setlength{\voffset}{-0.75in}
\setlength{\headsep}{5pt}
% add the \email cmd 
\newcommand\email[1]{\href{mailto:#1}{\nolinkurl{#1}}}

\PassOptionsToPackage{no-math}{fontspec}
\ifdefstring{\ELEGANT@lang}{cn}{
\RequirePackage[UTF8, scheme=plain]{ctex}
}

\ifxetex
  \RequirePackage{fontenc}
\else
  \RequirePackage[T1]{fontenc}
  \AtEndOfClass{\RequirePackage{microtype}}
\fi
\RequirePackage{newtxtext}
\RequirePackage{newtxmath}


\ifdefstring{\ELEGANT@lang}{cn}{
    \theoremstyle{plain}% default
	\newtheorem{theorem}{定理}[section] %
	\newtheorem{lemma}[theorem]{引理} %
	\newtheorem{proposition}[theorem]{性质} %
	\newtheorem*{corollary}{推论} %
	\theoremstyle{definition} %
	\newtheorem{definition}{定义}[section] %
	\newtheorem{conjecture}{猜想}[section] %
	\newtheorem{example}{例}[section] %
	\theoremstyle{remark} %
	\newtheorem*{remark}{\normalfont\bfseries 评论} %
	\newtheorem*{note}{\normalfont\bfseries 注} %
	\newtheorem{case}{\normalfont\bfseries 案例} %
	\renewcommand*{\proofname}{\normalfont\bfseries 证明} %
	\renewcommand\contentsname{目录}
	\renewcommand\refname{参考文献} %
	\renewcommand\figurename{图} %
	\renewcommand\tablename{表}%
	\newcommand\versiontext{版本：}%
	\newcommand\updatetext{日期：}%
	\renewcommand\abstractname{摘\hspace{2em}要}
	\setlength{\parindent}{2em}
	\renewcommand{\abstracttextfont}{\small\sffamily\kaishu\noindent\ignorespaces}
	\newcommand{\keywords}[1]{\vskip2ex\par\noindent\normalfont{\bfseries 关键词: } #1}
	\newcommand\figref[1]{{\bfseries 图~\ref{#1}}}
	\newcommand\tabref[1]{{\bfseries 表~\ref{#1}}}
%	\RequirePackage{gbt7714}
}{\relax}
\ifdefstring{\ELEGANT@lang}{en}{
  \theoremstyle{plain}% default
	\newtheorem{theorem}{Theorem}[section] %
	\newtheorem{lemma}[theorem]{Lemma} %
	\newtheorem{proposition}[theorem]{Proposition} %
	\newtheorem*{corollary}{Corollary} %
	\theoremstyle{definition} %
	\newtheorem{definition}{Definition}[section] %
	\newtheorem{conjecture}{Conjecture}[section] %
	\newtheorem{example}{Example}[section] %
	\theoremstyle{remark} %
	\newtheorem*{remark}{Remark} %
	\newtheorem*{note}{Note} %
	\newtheorem{case}{Case} %
	\renewcommand*{\proofname}{\normalfont\bfseries Proof}%
	\newcommand\versiontext{\itshape Version: }%
	\newcommand\updatetext{\itshape Date: }%
	\renewcommand{\abstracttextfont}{\sffamily\small\noindent\ignorespaces}
	\newcommand{\keywords}[1]{\vskip2ex\par\noindent\normalfont{\bfseries Keywords: } #1}
	\newcommand\figref[1]{{\bfseries Figure~\ref{#1}}}
	\newcommand\tabref[1]{{\bfseries Table~\ref{#1}}}
  	\bibliographystyle{aer}
}{\relax}

\AtBeginEnvironment{verbatim}{\microtypesetup{activate=false}}

\AtBeginDocument{
\setlength{\abovedisplayskip}{3pt}
\setlength{\belowdisplayskip}{3pt}
\RequirePackage[flushmargin]{footmisc}
\setlength{\footnotesep}{12pt}
}

\BeforeBeginEnvironment{tabular}{\small}
\AfterEndEnvironment{tabular}{}



\def\IfEmpty#1{%
  \edef\1{\the#1}
  \ifx\1\empty
}
\newtoks\version
\newtoks\institute
\def \ifempty#1{\def\temp{#1} \ifx\temp\empty }
\renewcommand\maketitle{\par
  \begingroup
    \renewcommand\thefootnote{\@fnsymbol\c@footnote}%
    \def\@makefnmark{\rlap{\@textsuperscript{\normalfont\@thefnmark}}}%
    \long\def\@makefntext##1{\parindent 1em\noindent
            \hb@xt@0.1em{%
                \hss\@textsuperscript{\normalfont\@thefnmark}}##1}%
    \if@twocolumn
      \ifnum \col@number=\@ne
        \@maketitle
      \else
        \twocolumn[\@maketitle]%
      \fi
    \else
      \newpage
      \global\@topnum\z@   % Prevents figures from going at top of page.
      \@maketitle
    \fi
    \thispagestyle{plain}\@thanks
  \endgroup
  \setcounter{footnote}{0}%
  \global\let\thanks\relax
  \global\let\maketitle\relax
  \global\let\@maketitle\relax
  \global\let\@thanks\@empty
  \global\let\@author\@empty
  \global\let\@ELEGANT@version\@empty
  \global\let\@date\@empty
  \global\let\@title\@empty
  \global\let\title\relax
  \global\let\author\relax
  \global\let\date\relax
  \global\let\and\relax
  \global\let\ELEGANT@version\relax
}
\def\@maketitle{%
  \newpage
  \null
  \vskip 2em%
  \begin{center}%
  \let \footnote \thanks
    {\LARGE\bfseries \@title \par}%
    \vskip 1.5em%
    {\large
      \lineskip .5em%
      \begin{tabular}[t]{c}%
        \@author\\
        \the\institute
      \end{tabular}\par}%
\vskip 1em%
\IfEmpty\version
\else 
{\small\normalfont\ifdefstring{\ELEGANT@lang}{en}{\itshape}{\kaishu}\versiontext\the\version}
\fi
\ifx\@date\empty
\else
\vskip 0.1em%
 {\small\normalfont\ifdefstring{\ELEGANT@lang}{en}{\itshape}{\kaishu}\updatetext\@date}%
\fi
  \end{center}%
  \par
}
  



\DeclareSymbolFont{CMletters}     {OML}{cmm} {m}{it}%
\DeclareSymbolFont{CMsymbols}     {OMS}{cmsy}{m}{n}%
\DeclareSymbolFont{CMlargesymbols}{OMX}{cmex}{m}{n}%

% \let\sum\relax
% \DeclareMathSymbol{\sum}{\mathop}{CMlargesymbols}{"50}%
% \let\int\relax
% \DeclareMathSymbol{\int}{\mathop}{CMlargesymbols}{"5A}%

\let\intop\relax\let\sumop\relax\let\ointop\relax\let\prodop\relax
\DeclareMathSymbol{\sumop}{\mathop}{CMlargesymbols}{"50}
\DeclareMathSymbol{\intop}{\mathop}{CMlargesymbols}{"52}
\DeclareMathSymbol{\ointop}{\mathop}{CMlargesymbols}{"49}
\DeclareMathSymbol{\prodop}{\mathop}{CMlargesymbols}{"59}

% \let\partial\relax
% \DeclareMathSymbol{\partial}{\mathord}{CMletters}{"40}%
% \let\in\relax
%\DeclareMathSymbol{\in}{\mathrel}{CMsymbols}{"32}%
% \let\pi\relax
% \DeclareMathSymbol{\pi}{\mathord}{CMletters}{"19}



\RequirePackage{listings} 
\renewcommand{\ttdefault}{cmtt}
\lstdefinestyle{mystyle}{
  basicstyle=%
    \ttfamily
    \lst@ifdisplaystyle\footnotesize\fi
}

\lstset{basicstyle=\scriptsize\ttfamily,style=mystyle}

\definecolor{lightgrey}{rgb}{0.9,0.9,0.9}
\definecolor{frenchplum}{RGB}{190,20,83}
\lstset{language=[LaTeX]TeX,
	texcsstyle=*\color{winered},
	numbers=none,
	breaklines=true,
	keywordstyle=\color{winered},
	commentstyle=\color{gray},
	emph={elegantpaper,fontenc,fontspec,xeCJK,FiraMono,xunicode,newtxmath,figure,fig,image,img,table,itemize,enumerate,newtxtext,newtxtt,ctex,microtype,description,times,newtx,booktabs,tabular,PDFLaTeX,XeLaTeX,type1cm,BibTeX},
	emphstyle={\color{frenchplum}},
	morekeywords={DeclareSymbolFont,SetSymbolFont,toprule,midrule,bottomrule,institute,version,includegraphics,setmainfont,setsansfont,setmonofont ,setCJKmainfont,setCJKsansfont,setCJKmonofont,RequirePackage,figref,tabref,email,maketitle,keywords},
	frame=none,
	tabsize=2,
	backgroundcolor=\color{lightgrey}
}




