package org.firstinspires.ftc.teamcode;

public final class BallQueueChecks {
    private static void expect(BallQueue<String> queue, String expected) {
        if (!expected.equals(queue.toString())) {
            throw new AssertionError("Expected " + expected + ", got " + queue);
        }
    }

    public static void main(String[] args) {
        BallQueue<String> queue = new BallQueue<>("POLLEN", 1.0);
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(false, false, 0, "NECTAR");
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, false, 0.1, "NECTAR");
        expect(queue, "[POLLEN, NECTAR]");
        queue.update(true, false, 5, null);
        expect(queue, "[POLLEN, NECTAR]");
        queue.update(true, true, 6, null);
        queue.update(true, true, 6.9, null);
        expect(queue, "[POLLEN, NECTAR]");
        queue.update(true, true, 7, null);
        expect(queue, "[NECTAR, POLLEN]");
        if (!"NECTAR".equals(queue.oldest())) {
            throw new AssertionError("The oldest ball must drive the throat after one advance");
        }
        queue.update(false, true, 7.5, "NECTAR");
        queue.update(true, true, 8, null);
        queue.update(true, true, 8.9, null);
        expect(queue, "[NECTAR, POLLEN]");
        queue.update(true, true, 9, null);
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, true, 9.1, "NECTAR");
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, true, 9.2, null);
        queue.update(true, true, 9.3, "POLLEN");
        queue.update(true, true, 9.4, null);
        queue.update(true, true, 9.45, "NECTAR");
        expect(queue, "[POLLEN, NECTAR]");
        queue.update(true, false, 9.5, null);
        queue.update(true, true, 10, null);
        queue.update(true, true, 10.9, null);
        expect(queue, "[POLLEN, NECTAR]");
        queue.update(true, true, 11, null);
        expect(queue, "[NECTAR, POLLEN]");

        queue.reset();
        queue.update(true, true, 20, "NECTAR");
        queue.update(true, true, 21, "NECTAR");
        expect(queue, "[NECTAR, POLLEN]");
        queue.update(true, true, 22, "NECTAR");
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, true, 23, "NECTAR");
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, true, 23.1, null);
        queue.update(true, true, 23.2, "NECTAR");
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, true, 23.3, null);
        queue.update(true, true, 23.4, "POLLEN");
        queue.update(true, true, 23.5, null);
        queue.update(true, true, 23.6, "NECTAR");
        expect(queue, "[POLLEN, NECTAR]");
        queue.update(true, true, 26.5, null);
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, true, 26.6, "NECTAR");
        queue.update(true, true, 27, null);
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(false, true, 27.1, "POLLEN");
        queue.update(true, true, 27.2, "NECTAR");
        expect(queue, "[POLLEN, POLLEN]");
        queue.reset();
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, false, 30, "NECTAR");
        expect(queue, "[POLLEN, NECTAR]");
        queue.update(true, false, 30.1, null);
        queue.update(true, false, 30.2, "POLLEN");
        expect(queue, "[POLLEN, POLLEN]");
        queue.update(true, false, 30.3, null);
        queue.update(true, false, 30.4, "NECTAR");
        expect(queue, "[POLLEN, NECTAR]");
    }
}
