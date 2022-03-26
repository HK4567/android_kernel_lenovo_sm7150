ANYKERNEL3_DIR=$PWD/Anykernel3
FINAL_KERNEL_ZIP=Marshmallow_v0.3-$(git rev-parse --short=7 HEAD).zip
IMAGE_GZ=$PWD/out/arch/arm64/boot/Image.gz

cp $IMAGE_GZ $ANYKERNEL3_DIR/Image.gz
cd $ANYKERNEL3_DIR/
zip -r $FINAL_KERNEL_ZIP *
cd ..
rm $ANYKERNEL3_DIR/Image.gz
mv -f $ANYKERNEL3_DIR/$FINAL_KERNEL_ZIP $PWD/
