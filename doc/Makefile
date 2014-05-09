
all: www/index.html www/api.html www/tutorials.html www/downloads.html www/renderer.html ../readme.txt ../readme.pdf 

replace=sed \
-e '/<CONTENT_OVERVIEW>/{r src/overview.html' -e 'd' -e '}' \
-e '/<CONTENT_NEW_FEATURES>/{r src/new_features.html' -e 'd' -e '};' \
-e '/<CONTENT_SUPPORT>/{r src/support.html' -e 'd' -e '};' \
-e '/<CONTENT_PLATFORMS>/{r src/platforms.html' -e 'd' -e '};' \
-e '/<CONTENT_DOWNLOADING>/{r src/downloading.html' -e 'd' -e '};' \
-e '/<CONTENT_COMPILATION>/{r src/compilation.html' -e 'd' -e '};' \
-e '/<CONTENT_API>/{r src/api.html' -e 'd' -e '};' \
-e '/<CONTENT_TUTORIALS>/{r src/tutorials.html' -e 'd' -e '};' \
-e '/<CONTENT_RENDERER>/{r src/renderer.html' -e 'd' -e '};' \
$(1) > $(2)

www/index.html: src/*.html src/www/frame_index.html
	$(call replace,src/www/frame_index.html,www/index.html)

www/api.html: src/*.html src/www/frame_api.html
	$(call replace,src/www/frame_api.html,www/api.html)

www/tutorials.html: src/*.html src/www/frame_tutorials.html
	$(call replace,src/www/frame_tutorials.html,www/tutorials.html)

www/downloads.html: src/*.html src/www/frame_downloads.html
	$(call replace,src/www/frame_downloads.html,www/downloads.html)

www/renderer.html: src/*.html src/www/frame_renderer.html
	$(call replace,src/www/frame_renderer.html,www/renderer.html)

readme.html: src/*.html
	$(call replace,src/readme.html,readme.html)

../readme.txt: readme.html
	html2text -rcfile html2text.conf readme.html > readme.txt
	cp readme.txt ..

../readme.pdf: readme.html
	html2ps -t -f html2ps.conf readme.html > readme.ps
	ps2pdf readme.ps 
	cp readme.pdf ..

clean:
	rm readme.txt
	rm readme.pdf
	rm readme.ps
	rm readme.html
	rm www/*.html
