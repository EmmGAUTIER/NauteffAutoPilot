LATEXMK = latexmk -pdf -interaction=nonstopmode -file-line-error

SRC_DIR   = src
PDF_DIR   = pdf
AUX_DIR   = aux

MAINS = rapport annexe

all: $(MAINS:%=$(PDF_DIR)/%.pdf)

$(PDF_DIR)/%.pdf: $(SRC_DIR)/%.tex
	mkdir -p $(PDF_DIR) $(AUX_DIR)/$*
	$(LATEXMK) \
	  -output-directory=$(PDF_DIR) \
	  -aux-directory=$(AUX_DIR)/$* \
	  $<

clean:
	rm -rf $(AUX_DIR)/*

mrproper: clean
	rm -f $(PDF_DIR)/*.pdf

.PHONY: all clean mrproper

