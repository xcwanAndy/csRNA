" ============================
" Copy this file to project's root path and rename, e.g.,
" $ cp ~/.vim/prjconf.d/cust.vim prj_path/.cust.vim
" ============================


" ====== Tag settings ======
" Customize tags for C language.
"set tags+=/usr/include/c++/5/stdcpp.tags
set tags+=~/.vim/tags/include_tags
"set tags+=~/mb/metls/ctag_metls


" ====== Tab settings ======
"Expand tab to space
"set expandtab
"Set space number of tab when edit
"set tabstop=4
"Set space number of tab when format
"set shiftwidth=4
"Continued space number that can be considered as tab
"set softtabstop=4


" ====== Check settings ======
" Customize whitespace checks for C.
" useful to call for particular file types (e.g., in "ftplugin/*")
"silent! call airline#extensions#whitespace#disable()


" TEST
" Commend this line after succeed
"echo "This is a test of cust.vim"
"

" Gem5 code style
filetype indent on "auto indenting
set tabstop=4 "tabs = 4 spaces
set shiftwidth=4 "auto indent = 4 spaces
set expandtab "expand tabs to spaces
set tw=78 "max cols is 78

" highlight extrawhite space with light blue background
highlight ExtraWhitespace ctermbg=lightblue guibg=lightblue
match ExtraWhitespace /\s\+$\|\t/

" stuff to prevent the light blue highlighting from showing up at the end of
" lines when you're in insert mode. i.e., everytime you enter a space as you're
" entering text the highlighting will kick in, which can be annoying. this will
" make the highlighting only show up if you finish editing and leave some extra
" whitespace
autocmd BufWinEnter * match ExtraWhitespace /\s\+$\|\t/
autocmd InsertEnter * match ExtraWhitespace /\s\+\%#\@<!$\|\t\%#\@<!/
autocmd InsertLeave * match ExtraWhitespace /\s\+$\|\t/
autocmd BufWinLeave * call clearmatches()


" optionally set a vertical line on column 79. anything on, or after the line
" is over the limit. this can be useful as set tw=78 won't breakup existing
" lines that are over the limit, and the user can also do certain things to
" make lines go past the set textwidth, e.g., joining a line (shift-j or J)

"if exists('+colorcolumn')
"    set colorcolumn=79
"endif


" optionally set spell checking
"set spell

" optionally highlight whitespace with specified characters. tab for trailing
" tabs, trail for trailing whitespace, extends for lines that extend beyond
" screen when wrap is off, and non-breakable white spaces. list must be set
" for these characters to display.
"set list
"set listchars=tab:›\ ,trail:•,extends:#,nbsp:.
