# Copyright 1999-2017 Gentoo Foundation
# Distributed under the terms of the GNU General Public License v2

EAPI=5

inherit linux-info linux-mod

if [ "${PV}" = "9999" ]; then
	inherit git-2
	EGIT_REPO_URI="https://github.com/josch09/au6601.git"
	KEYWORDS="~amd64"
else
	inherit vcs-snapshot
	SRC_URI="https://github.com/josch09//${PN}/archive/v${PV}.tar.gz -> ${P}.tar.gz"
	KEYWORDS="~amd64"
fi

DESCRIPTION="Driver for PCI driver for Alcor Micro AU6601 Secure Digital Host Controller Interface"
HOMEPAGE="https://launchpadlibrarian.net/175159219/[PATCH] mmc%3A add new au6601 driver.eml"

LICENSE="GPL-2"
SLOT="0"
IUSE=""

CONFIG_CHECK=""
MODULE_NAMES="au6601-pci(misc:${S})"
BUILD_TARGETS="all"

src_compile(){
	BUILD_PARAMS="KDIR=${KV_OUT_DIR} M=${S}"
	linux-mod_src_compile
}
