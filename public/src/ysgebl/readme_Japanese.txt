Polygon Editor - Polygon Crest
By Captain YS
http://www.ysflight.com




[�͂��߂�]

�uPolygon Crest�v�̓|���S���G�f�B�^�i�|���S�����f���ƌĂԕ�����ʓI�����j�ŁA3�����|���S�����f�����쐬�E�ҏW���邱�Ƃ��ł���B���Ƃ��Ƃ̖ړI��YS FLIGHT SIMULATOR�p�̔�s�@�E�n�㕨�`���ҏW���邽�߂ɊJ�������v���O���������ASTL�`���̏o�͂��\�Ȃ̂ŁA���Ƃ��Η��̂Ƃ��Đ������f�[�^�����΁A�O�����v�����^�p�̃f�[�^���o�͂��邱�Ƃ��ł���B��s�@���f�������₷���悤�Ɉӎ����Ă���̂ŁA��\�I�ȗ��f�ʌn���ȒP�ɍ쐬����@�\��A������A�����ʌn�ɉ����ăX�C�[�v�����`���쐬���邱�Ƃ��\�B�܂��A�W�����Z�A�ȒP�Ȋۂߋ@�\�����ڂ��Ă���B

�g�����́A����URL�ŎQ�Ƃ��邱�Ƃ��ł���B

http://polycre.help.jp.ysflight.com/   (���{���)
http://polycre.help.en.ysflight.com/   (�p���)

�����A�����͂��Ȃ菑�������Ȃ̂ŁA�ق�̈ꕔ�̋@�\�����J�o�[�ł��Ă��Ȃ��B�c��́A���ꂩ�珇���ǉ����Ă����\��ł���B

20150810�o�[�W��������AMacOSX��Linux�̃o�C�i���p�b�P�[�W�𓝈ꂵ�܂����BLinux��ł̎��s���@�́A�ȉ��́u�v���O�����̋N���v���Q�Ƃ��Ă��������B
20160220�o�[�W��������ALinux�p�C���X�g�[�� (Python script) �������ɓ������܂����B




[��t�ɂ���]
�������A1000�~���炢��t���Ă����܂�Ȃ��A�Ƃ����������܂�����A�ȉ�URL�����1000�~�ł���������������Ƒ�ϊ��ӂ������܂��B

    http://s.shop.vector.co.jp/service/servlet/NCart.Add?ITEM_NO=SR360800

�������������������z�́A�J�����̃A�b�v�O���[�h�A�����̍w���A���̑�YSFLIGHT.COM�̃����e�i���X�A�R���e���c�쐬�̔�p�Ɏg�킹�Ă��������܂��B��낵�����肢���܂��B





[�v���O�����̋N��]
MacOSX�̏ꍇ -> �_�E�����[�h����Zip���𓀂��� bin �f�B���N�g����PolygonCrest���_�u���N���b�N(64�r�b�g�ł̂�)

Linux�̏ꍇ -> �_�E�����[�h����Zip���𓀂��� "bin/Polygon Crest.app/Contents/Resources/ysgebl32" (32�r�b�g��)�܂��́A"bin/Polygon Crest.app/Contents/Resources/ysgebl64" (64�r�b�g��)���_�u���N���b�N�B(���邢�́ALinuxInstaller.py���_�u���N���b�N���Ď��s����ƃf�X�N�g�b�v�ɃA�C�R�����ł���̂ŁA���̃A�C�R��������N���\�B)

Windows�̏ꍇ  -> �_�E�����[�h����Zip���𓀂��� bin �f�B���N�g����ysgebl32.exe (32�r�b�g��)�܂��� ysgebl64.exe (64�r�b�g��)���_�u���N���b�N�B





[�\�[�X�R�[�h]
�\�[�X�R�[�h��Github����_�E�����[�h�ł���B�R���p�C������ɂ́A���̃X�e�b�v�ŃR�}���h����͂���B

git clone https://github.com/captainys/public
mkdir build
cd build
cmake ../public/src
history
cmake --build . --config Release

���̃R�}���h�́Abuild�f�B���N�g���� ysgebl/src/main/Release �T�u�f�B���N�g���Ɏ��s�t�@�C���𐶐�����B





[���C�Z���X�ɂ���]

�Ƃ肠�����A20160220�łƂ���ȑO�̃o�[�W�����́ABSD���C�Z���X�Ńt���[�B����̃o�[�W�����ł͕ς�邩������Ȃ��B

�Ȃ��ABSD���C�Z���X�̏����͈ȉ����Q�ƁB

/* ////////////////////////////////////////////////////////////

PolygonCrest
Copyright (c) 2014 Soji Yamakawa.  All rights reserved.
http://www.ysflight.com

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation 
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//////////////////////////////////////////////////////////// */
