spinn_aer.pdf: spinn_aer.tex appnote.tex cochlea_map.pdf\
	retina_maps.pdf map3.pdf out_map2.pdf spinn_2_if_diag.pdf

	pdflatex spinn_aer
	pdflatex spinn_aer

%.pdf: %.fig
	fig2dev -Lpdf $< $@

clean:
	/bin/rm -f *.aux *.auxlock *.bak *.bbl *blg *log *pdf *~
